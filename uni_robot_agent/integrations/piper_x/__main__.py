"""Explicit instance deployment and offline configuration validation."""
import argparse
import json
from pathlib import Path


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    commands.add_parser("camera-inventory", help="Read SDK and USB identities without starting camera streams")
    for name in ("validate", "host-config", "environment"):
        sub = commands.add_parser(name)
        sub.add_argument("--deployment", type=Path, required=True)
        sub.add_argument("--execute", action="store_true", help="Require complete per-station motion acceptance")
        if name == "host-config":
            sub.add_argument("--output", type=Path, required=True)
            sub.add_argument("--also", type=Path, action="append", default=[])
        if name == "environment":
            sub.add_argument("--output-dir", type=Path, required=True)
            sub.add_argument("--run-id", required=True)
            sub.add_argument("--instruction", required=True)
            sub.add_argument("--host", required=True)
            sub.add_argument("--port", required=True, type=int)
    args = parser.parse_args(argv)
    if args.command == "camera-inventory":
        from .hardware.camera_identity import inventory
        print(json.dumps(inventory(), indent=2))
        return
    from .config import Installation
    installation = Installation(args.deployment)
    if args.execute:
        installation.require_motion()
    if args.command == "validate":
        installation.environment(execute=args.execute)
        print(json.dumps({**installation.audit(), "hardware_accessed": False}, indent=2))
    elif args.command == "host-config":
        from .hardware.kit import host_config
        from uni_robot_agent.artifacts.run import write_json_atomic
        from uni_robot_agent.integrations.piper_x.body.host.config import validate_host
        data = host_config([installation, *(Installation(p) for p in args.also)], execute=args.execute)
        if args.output.exists():
            raise ValueError("Choose a new host output file; existing deployments are not overwritten")
        validate_host(data, args.output)
        write_json_atomic(args.output, data)
        print(json.dumps({"config": str(args.output), "instances": list(data["bodies"]), "hardware_accessed": False}))
    else:
        if args.host not in {"127.0.0.1", "localhost"}:
            raise ValueError("Physical environment routes must bind loopback; use an authenticated SSH tunnel remotely")
        if not 1024 <= args.port <= 65535 or args.port in {b.port for b in installation.host.bodies.values()}:
            raise ValueError("Environment port is invalid or conflicts with a Body")
        from .environment.server import build_service, serve
        service = build_service(installation, args.output_dir, episode_id=args.run_id, instruction=args.instruction, execute=args.execute)
        serve(service, args.output_dir, host=args.host, port=args.port)


if __name__ == "__main__":
    main()
