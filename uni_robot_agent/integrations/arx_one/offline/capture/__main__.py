"""Standalone read-only observation capture and archive inspection."""
from __future__ import annotations

import argparse
from importlib.resources import files
import json
import sys

from ..fleet import Fleet, load_site, site_from_dict
from ...perception.observation_store import ObservationStore
from .observe import ObservationService
from .regions import DepthService


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    for name in ("demo", "capture", "list", "get", "events", "filter-depth", "back-project", "complete-depth"):
        cmd = sub.add_parser(name)
        cmd.add_argument("--output", required=True, help="Observation archive directory")
        if name == "capture":
            cmd.add_argument("--site", required=True)
            cmd.add_argument("--robot", required=True)
            cmd.add_argument("--camera", action="append")
            cmd.add_argument("--timeout-s", type=float, default=3.0)
        if name == "get":
            cmd.add_argument("observe_id")
            cmd.add_argument("--raw", action="store_true")
        if name in {"filter-depth", "back-project", "complete-depth"}:
            cmd.add_argument("observe_id")
            cmd.add_argument("--camera", required=True)
        if name == "complete-depth":
            cmd.add_argument("--depth-config", required=True)
        if name == "back-project":
            selection = cmd.add_mutually_exclusive_group(required=True)
            selection.add_argument("--segmentation-artifact")
            selection.add_argument("--pixels", type=json.loads, help='JSON [[row,col], ...]')
            selection.add_argument("--region", type=json.loads, help='JSON {"box":[top,left,bottom,right]}')
            cmd.add_argument("--depth-source", choices=["measured", "filtered", "model", "filled"], default="filled", help="Depth source (default: filled; requires --depth-artifact)")
            cmd.add_argument("--depth-artifact")
            cmd.add_argument("--calibration", help="Binding frozen in this observation")
        if name == "events":
            cmd.add_argument("--after", type=int, default=0)
    args = parser.parse_args(argv)
    store = ObservationStore(args.output)
    if args.command == "complete-depth":
        from ...perception.completion.client import CompletionService, create_depth_backend
        # The same worker start as the services: a local worker runs worker_entry.py in its model environment.
        service = CompletionService(store, create_depth_backend(args.depth_config, store.root / 'worker'))
        try: result = service.complete(args.observe_id,args.camera)
        finally: service.close()
    elif args.command == "filter-depth":
        result = DepthService(store).filter_depth(args.observe_id, args.camera)
    elif args.command == "back-project":
        result = DepthService(store).back_project(args.observe_id, args.camera,
            pixels=args.pixels, region=args.region, segmentation_artifact=args.segmentation_artifact, depth_source=args.depth_source,
            depth_artifact=args.depth_artifact, calibration=args.calibration)
    elif args.command == "list":
        result = store.list()
    elif args.command == "get":
        result = store.get(args.observe_id, include_raw=args.raw)
    elif args.command == "events":
        result = store.events(after=args.after)
    else:
        if args.command == "demo":
            site = site_from_dict(json.loads(files("uni_robot_agent.integrations.arx_one.offline.fleet").joinpath("fake_observation_site.json").read_text()))
            robot_id, cameras, timeout = "demo", None, 3.0
        else:
            site = load_site(args.site)
            robot_id, cameras, timeout = args.robot, args.camera, args.timeout_s
        fleet = Fleet(site)
        try:
            fleet.connect()
            clock_id = "linux-boot:synthetic-observation-fixture" if args.command == "demo" else None
            with ObservationService(fleet.robot(robot_id), store, clock_id=clock_id) as service:
                result = service.observe(cameras, timeout_s=timeout)
        finally:
            fleet.close()
    print(json.dumps(result, indent=2, allow_nan=False))
    return 0 if not isinstance(result, dict) or result.get("status", "completed") == "completed" else 2


def run(argv=None):
    """The command line entry: expected failures print one line instead of a traceback."""
    try:
        return main(argv)
    except (ValueError, KeyError, OSError, RuntimeError, TimeoutError) as exc:
        print(f"offline: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(run())
