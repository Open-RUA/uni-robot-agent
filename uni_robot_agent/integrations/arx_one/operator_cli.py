"""Operator commands for one robot host: Bodies, their controllers and manual arm actions.
Agent tools never call these.

Start order is controllers, then Bodies; stop order is the reverse. Starting a
controller is a physical action: its start-up can home an arm and close its
gripper. Every command reports evidence and none of them retries motion.
"""
import argparse
from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import signal
import threading
import subprocess
import sys
import time

from .config import BodyConfig, Config
from .environment.control import BodyClient, BodyError
from .environment.control import Control, confirm_unowned_idle
from .environment.native_compat import verify
from .body.core.arm import contract
from .body.core.camera import contract as camera_contract
from .body.core.contract import TERMINAL_STATES

WRAPPER = Path(__file__).parent / "deploy" / "scripts" / "rua-host-local.sh"
REQUIRED_CHECKS = ("feedback", "motors", "can_traffic", "joint_envelope", "controller")
STABLE_SAMPLES = 7
STABLE_RAD = .005
HOME_ACCEPT_RAD = .02
GRIPPER_ACCEPT = .08


class OperatorError(RuntimeError):
    pass


def bodies(config):
    return {"arms": config.arms_body, "cameras": config.cameras_body}


def health_of(config):
    """Report each Body independently; an offline Body is a normal status."""
    report = {}
    for name, body in bodies(config).items():
        client = BodyClient(body, timeout_s=config.rpc_timeout_s)
        try:
            report[name] = {"online": True, **client.call("get_health")}
        except Exception as error:
            report[name] = {"online": False, "error": f"{type(error).__name__}: {error}"[:200]}
        finally:
            client.close()
    return report


def require_quiet(health, body_id):
    if health.get("body_id") != body_id:
        raise OperatorError(f"Body identity changed: {health.get('body_id')}")
    busy = {key: health.get(key) for key in ("leases", "active_operations", "open_streams") if health.get(key) != 0}
    if busy:
        raise OperatorError(f"Body still has control or work: {busy}")


@dataclass
class Host:
    """How this CLI reaches the installed runtime without changing the selected release."""
    runtime_root: Path
    config: Path
    arms_instance: str
    cameras_instance: str
    # systemd unit names are <prefix>-body-<instance> and <prefix>-controllers-<instance>-<time>.
    unit_prefix: str = "rua"

    def command(self, *args):
        return [str(WRAPPER), str(self.runtime_root), str(self.config), *args]

    def unit(self, instance):
        return f"{self.unit_prefix}-body-{instance}.service"

    def run(self, *args, scope=None):
        command = self.command(*args)
        if scope:
            # Controllers outlive this command. In their own scope, stopping or
            # restarting whatever launched them (a service, a terminal) leaves them running.
            command = ["systemd-run", "--user", "--scope", "--collect", "--quiet", f"--unit={scope}", *command]
        subprocess.run(command, check=True)

    def start_body(self, instance):
        # KillMode=process leaves native helpers to their own stop handling.
        subprocess.run(["systemd-run", "--user", f"--unit={self.unit(instance)}", "--collect",
                        "--property=KillMode=process", "--property=TimeoutStopSec=25",
                        *self.command("body", "--config", str(self.config), "--instance", instance)], check=True)

    def stop_body(self, instance):
        subprocess.run(["systemctl", "--user", "stop", self.unit(instance)], check=True)


def wait_online(config, name, timeout_s=60.):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        report = health_of(config)[name]
        if report["online"]:
            return report
        time.sleep(1.)
    raise OperatorError(f"{name} Body did not come online within {timeout_s:g}s")


def up(config, host):
    """Start what is missing: controllers, then any Body not already serving.

    A serving Body is reused only when it is this installation's Body and quiet.
    """
    report = health_of(config)
    for name, body in bodies(config).items():
        if report[name]["online"]:
            require_quiet(report[name], body.body_id)
    host.run("controllers", "start", "--config", str(host.config), "--instance", host.arms_instance,
             scope=f"{host.unit_prefix}-controllers-{host.arms_instance}-{time.strftime('%Y%m%d%H%M%S')}")
    for name, instance in (("arms", host.arms_instance), ("cameras", host.cameras_instance)):
        if not report[name]["online"]:
            host.start_body(instance)
            wait_online(config, name)
    return health_of(config)


def down(config, host):
    report = health_of(config)
    for name, body in bodies(config).items():
        if report[name]["online"]:
            require_quiet(report[name], body.body_id)
    for instance in (host.cameras_instance, host.arms_instance):
        host.stop_body(instance)
    host.run("controllers", "stop", "--config", str(host.config), "--instance", host.arms_instance)
    return {"stopped": True}


def identity(path):
    """What each arm and camera Body reports for the commissioning fields of environment.json; reads only.

    Works on a configuration that is not complete yet, so it reads only the Body sections and each
    device's ``device_id`` and pinned fields.
    """
    raw = json.loads(Path(path).read_text())
    timeout_s = raw.get("rpc_timeout_s", 2.)
    arms = _reported(raw["arms_body"], raw.get("arms") or {}, timeout_s, contract.problems, arm=True)
    cameras = _reported(raw["cameras_body"], raw.get("cameras") or {}, timeout_s, camera_contract.problems)
    return {"arms": arms, "cameras": cameras}


def _reported(body_section, devices, timeout_s, problems, *, arm=False):
    if not devices:
        return {}
    client = BodyClient(BodyConfig.model_validate(body_section), timeout_s=timeout_s)
    try:
        description = client.call("describe", {"device_ids": [d["device_id"] for d in devices.values()]})
    finally:
        client.close()
    found = {device["device_id"]: device for device in description["devices"]}
    report = {"body_id": description.get("body_id")}
    for name, configured in devices.items():
        device = found.get(configured["device_id"])
        if device is None:
            report[name] = {"error": f"Body has no device {configured['device_id']}"}
            continue
        metadata = device.get("metadata", {})
        reported = {"expected_metadata": {"identity": metadata.get("identity")}}
        current = {"expected_metadata": {"identity": (configured.get("expected_metadata") or {}).get("identity")}}
        if arm:
            limits = metadata.get("joint_limits_rad") or {}
            reported.update(joint_lower=limits.get("lower"), joint_upper=limits.get("upper"))
            current.update(joint_lower=configured.get("joint_lower"), joint_upper=configured.get("joint_upper"))
        report[name] = {**reported, "contract_problems": problems(device),
                        "differs_from_config": sorted(key for key in reported if reported[key] != current[key])}
        if arm:
            report[name]["model_sha256"] = metadata.get("model_sha256")
    return report


def single_arm(config, arm, timeout_s):
    """Scope one operation to one arm and shorten its action deadline."""
    if arm not in config.arms:
        raise OperatorError(f"Unknown arm {arm!r}; this robot has {sorted(config.arms)}")
    spec = config.arms[arm]
    return config.model_copy(update={"arms": {arm: spec}, "action_timeout_s": min(config.action_timeout_s, timeout_s)
                                     }), spec


def read_arm(client, device, max_age_s):
    return client.call("read_state", {"device_ids": [device], "freshness": {"max_age_s": max_age_s,
                                                                            "on_stale": "error"}})["devices"][device]


def finite_joints(reading, count):
    joints = reading.get("signals", {}).get("joint_position_rad")
    if (reading.get("freshness") != "fresh" or reading.get("error") or not isinstance(joints, list)
            or len(joints) != count or not all(math.isfinite(value) for value in joints)):
        raise OperatorError("Fresh joint feedback is unavailable")
    return joints


def _operate(config, client, spec, directory, action):
    """Run one bounded operation under a lease; the stop result is part of the receipt."""
    directory.mkdir(parents=True, exist_ok=True)
    confirm_unowned_idle(config, client)
    control = Control(client, [spec.device_id], config, directory / "body-journal.jsonl")
    stop = {"status": "not_required"}
    result = {"accepted": False}
    try:
        control.acquire()
        result.update(action(control))
        result["accepted"] = True
    except (Exception, KeyboardInterrupt) as error:
        result["error"] = str(error) or type(error).__name__
    finally:
        # A second signal must not abort the stop that confirms the arm is idle.
        ignored = {}
        if threading.current_thread() is threading.main_thread():
            ignored = {sig: signal.signal(sig, signal.SIG_IGN) for sig in (signal.SIGINT, signal.SIGTERM)}
        try:
            stop = control.stop("operator_finished")
            control.close()
        except Exception:
            stop = {"status": "unknown"}
        finally:
            for sig, handler in ignored.items():
                signal.signal(sig, handler)
    result["stop"] = stop
    result["accepted"] = result["accepted"] and stop.get("status") == "confirmed_idle"
    return result


def home(config, arm, directory, *, client=None):
    """One joint trajectory to zero with arrival checked once; never retried."""
    config, spec = single_arm(config, arm, 20.)
    if any(lower > 0 or upper < 0 for lower, upper in zip(spec.joint_lower, spec.joint_upper)):
        raise OperatorError("Zero is outside the commissioned joint limits")
    own = client is None
    client = client or BodyClient(config.arms_body, timeout_s=config.rpc_timeout_s)
    try:
        limits = verify(client, config, arm_names=[arm])[arm]
        initial = read_arm(client, spec.device_id, .3)
        joints = finite_joints(initial, len(spec.joint_names))
        if initial.get("moving") is not False:
            raise OperatorError("The arm is not stationary")
        duration = max(.2, 1.875 * max(abs(value) for value in joints) / limits["joint_speed_rad_s"])
        if duration + config.rpc_timeout_s >= config.action_timeout_s:
            raise OperatorError("The home path exceeds the bounded duration")

        def action(control):
            control.begin_action()
            operation = control.invoke(spec.device_id, "move_joints",
                                       {"kind": "absolute", "values": [0.] * len(spec.joint_names),
                                        "duration_s": duration})
            control.wait_operations([operation])
            control.end_action()
            measured = finite_joints(read_arm(client, spec.device_id, .3), len(spec.joint_names))
            if max(abs(value) for value in measured) > HOME_ACCEPT_RAD:
                raise OperatorError("Joint feedback does not match zero; not retrying")
            return {"operation_id": operation, "joint_position_rad": measured, "duration_s": duration}
        return _operate(config, client, spec, directory, action)
    finally:
        if own:
            client.close()


def set_gripper(config, arm, opening, directory, *, client=None):
    """One gripper command, accepted only on feedback newer than the command."""
    config, spec = single_arm(config, arm, 12.)
    own = client is None
    client = client or BodyClient(config.arms_body, timeout_s=config.rpc_timeout_s)

    def calibrated(reading):
        return reading.get("freshness") == "fresh" and contract.opening_measured(reading.get("signals", {}))
    try:
        verify(client, config, arm_names=[arm])
        if not calibrated(read_arm(client, spec.device_id, .5)):
            raise OperatorError("The gripper has no fresh calibrated feedback")

        def action(control):
            control.begin_action()
            operation = control.invoke(spec.device_id, "set_gripper", {"opening": opening, "duration_s": 1.})
            control.wait_operations([operation])
            control.end_action()
            boundary, deadline = time.time_ns(), time.monotonic() + 2.
            while time.monotonic() < deadline:
                control.require()
                reading = read_arm(client, spec.device_id, .5)
                measured = reading["signals"].get("gripper_opening")
                stamp = int((reading.get("timing") or {}).get("source_timestamp_ns") or 0)
                if (stamp > boundary and calibrated(reading) and type(measured) in (int, float)
                        and abs(measured - opening) <= GRIPPER_ACCEPT):
                    return {"operation_id": operation, "opening": measured}
                time.sleep(.03)
            raise OperatorError("Gripper feedback did not confirm the opening; not retrying")
        return _operate(config, client, spec, directory, action)
    finally:
        if own:
            client.close()


def calibrate_gripper(config, arm, directory, *, client=None):
    """Record one gripper's open and closed feedback with the Body's calibrate_gripper sweep."""
    config, spec = single_arm(config, arm, 60.)
    own = client is None
    client = client or BodyClient(config.arms_body, timeout_s=config.rpc_timeout_s)
    try:
        verify(client, config, arm_names=[arm])
        if read_arm(client, spec.device_id, .5).get("moving") is not False:
            raise OperatorError("The arm is not stationary")

        def action(control):
            control.begin_action()
            operation = control.invoke(spec.device_id, "calibrate_gripper", {})
            control.wait_operations([operation])
            control.end_action()
            reading = read_arm(client, spec.device_id, .5)
            if not contract.opening_measured(reading.get("signals", {})):
                raise OperatorError("The Body did not report calibrated gripper feedback; not retrying")
            return {"operation_id": operation, "opening": reading["signals"].get("gripper_opening")}
        return _operate(config, client, spec, directory, action)
    finally:
        if own:
            client.close()


def hardware_ready(check):
    """Quarantine itself fails readiness, so judge the hardware diagnostics apart from it."""
    checks = check.get("checks", {})
    return (all(checks.get(key, {}).get("state") == "pass" for key in REQUIRED_CHECKS)
            and all(item.get("state") == "pass" for key, item in checks.items()
                    if key not in {"quarantine", "gripper_calibration"}))


def acknowledge(config, arm, operation_ids, *, client=None, interval_s=.35):
    """Clear only the named finished operations after verifying a stable, quiet arm."""
    if arm not in config.arms:
        raise OperatorError(f"Unknown arm {arm!r}; this robot has {sorted(config.arms)}")
    spec = config.arms[arm]
    device = spec.device_id
    own = client is None
    client = client or BodyClient(config.arms_body, timeout_s=5.)
    try:
        health = client.call("get_health")
        require_quiet(health, config.arms_body.body_id)
        quarantine = set(health.get("quarantined", {}).get(device, []))
        if not quarantine:
            return {"acknowledged": [], "remaining_quarantine": []}
        if not quarantine <= set(operation_ids):
            raise OperatorError(f"Other unknown operations remain; name them after checking: {sorted(quarantine)}")
        for operation_id in quarantine:
            operation = client.call("get_operation", {"operation_id": operation_id})
            if operation.get("state") not in TERMINAL_STATES or device not in operation.get("device_ids", []):
                raise OperatorError(f"Operation {operation_id} has not finished on this arm; it stays quarantined")
        initial = None
        for sample in range(STABLE_SAMPLES):
            if not hardware_ready(client.call("check", {"device_ids": [device]})["devices"][device]):
                raise OperatorError("Hardware diagnostics do not pass; quarantine retained")
            reading = read_arm(client, device, .5)
            joints = finite_joints(reading, len(spec.joint_names))
            if reading.get("moving") is not False:
                raise OperatorError("The arm is moving; quarantine retained")
            initial = joints if initial is None else initial
            if max(abs(a - b) for a, b in zip(initial, joints)) > STABLE_RAD:
                raise OperatorError("Joint feedback is still changing; quarantine retained")
            if sample < STABLE_SAMPLES - 1:
                time.sleep(interval_s)
        health = client.call("get_health")
        require_quiet(health, config.arms_body.body_id)
        if not set(health.get("quarantined", {}).get(device, [])) <= set(operation_ids):
            raise OperatorError("New unknown operations appeared during verification; quarantine retained")
        result = client.call("acknowledge", {"device_ids": [device], "operation_ids": sorted(quarantine),
                                             "note": "Operator verified a stable, quiet arm; nothing replayed"})
        return {"acknowledged": sorted(quarantine),
                "remaining_quarantine": result.get("remaining_quarantine", {}).get(device, []),
                "joint_position_rad": joints}
    finally:
        if own:
            client.close()


def build_parser():
    parser = argparse.ArgumentParser(prog="operator_cli", description=__doc__)
    parser.add_argument("--config", type=Path, required=True, help="Environment configuration JSON")
    parser.add_argument("--output-dir", type=Path, default=Path("operator-journals"), help="Journals for manual actions")
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("status", help="Report Body identity, control, work and quarantine")
    sub.add_parser("identity", help="Print what each arm and camera Body reports for environment.json and how it differs")
    for name, text in (("up", "Start controllers, then Bodies (physical action)"),
                       ("down", "Stop Bodies, then controllers")):
        service = sub.add_parser(name, help=text)
        service.add_argument("--runtime-root", type=Path, default=os.environ.get("RUA_RUNTIME_ROOT"), required=False)
        service.add_argument("--host-config", type=Path, default=os.environ.get("RUA_HOST_CONFIG"), required=False)
        service.add_argument("--arms-instance", default="arms")
        service.add_argument("--cameras-instance", default="cams")
        service.add_argument("--unit-prefix", default="rua",
                             help="Prefix of the systemd units this command starts and stops")
        service.add_argument("--yes", action="store_true", help="Confirm the workspace is clear")
    move = sub.add_parser("home", help="Move one arm to joint zero (physical action)")
    move.add_argument("arm", help="Arm name from environment.json")
    move.add_argument("--yes", action="store_true", help="Confirm the path is clear")
    grip = sub.add_parser("gripper", help="Fully open or fully close one gripper (physical action)")
    grip.add_argument("arm", help="Arm name from environment.json")
    grip.add_argument("action", choices=("open", "close"))
    grip.add_argument("--yes", action="store_true", help="Confirm any held object is secured")
    calibrate = sub.add_parser("calibrate-gripper",
                               help="Sweep one gripper closed and open to record its feedback endpoints (physical action)")
    calibrate.add_argument("arm", help="Arm name from environment.json")
    calibrate.add_argument("--yes", action="store_true", help="Confirm nothing is between the fingers")
    ack = sub.add_parser("ack", help="Verify a stable arm, then clear named quarantined operations (admin token)")
    ack.add_argument("arm", help="Arm name from environment.json")
    ack.add_argument("--operation-id", action="append", default=[])
    ack.add_argument("--admin-token-env", required=True, help="Environment variable holding the admin token")
    return parser


def _interrupt(*_):
    raise KeyboardInterrupt("operator cancelled")


def main(argv=None):
    args = build_parser().parse_args(argv)
    signal.signal(signal.SIGTERM, _interrupt)  # A stopped command still stops the arm.
    if args.command in {"up", "down", "home", "gripper", "calibrate-gripper"} and not args.yes:
        raise SystemExit(f"{args.command} affects the physical robot; pass --yes once the workspace is clear")
    stamp = f"{args.command}-{time.strftime('%Y%m%d-%H%M%S')}"
    try:
        if args.command == "identity":
            print(json.dumps(identity(args.config), ensure_ascii=False, indent=2))
            return 0
        config = Config.load(args.config)
        if args.command == "status":
            result = health_of(config)
        elif args.command in {"up", "down"}:
            if not args.runtime_root or not args.host_config:
                raise OperatorError("Set --runtime-root and --host-config")
            host = Host(args.runtime_root.resolve(), args.host_config.resolve(), args.arms_instance,
                        args.cameras_instance, args.unit_prefix)
            result = up(config, host) if args.command == "up" else down(config, host)
        elif args.command == "home":
            result = home(config, args.arm, args.output_dir / stamp)
        elif args.command == "gripper":
            result = set_gripper(config, args.arm, 1. if args.action == "open" else 0., args.output_dir / stamp)
        elif args.command == "calibrate-gripper":
            result = calibrate_gripper(config, args.arm, args.output_dir / stamp)
        else:
            body = config.arms_body.model_copy(update={"token_env": args.admin_token_env})
            admin = config.model_copy(update={"arms_body": body})
            result = acknowledge(admin, args.arm, args.operation_id)
    except (OperatorError, BodyError, OSError, KeyError, ValueError, subprocess.CalledProcessError) as error:
        print(json.dumps({"error": str(error) or type(error).__name__}, ensure_ascii=False), file=sys.stderr)
        return 1
    print(json.dumps(result, ensure_ascii=False, indent=2, default=str))
    return 0 if result.get("accepted", True) else 1


if __name__ == "__main__":
    raise SystemExit(main())
