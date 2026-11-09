"""Validate a site manifest without connecting, or run the bundled in-memory fake robot."""
from __future__ import annotations

import argparse
import json
from importlib.resources import files

from . import Fleet, load_site, site_from_dict


def demo():
    """Use only the bundled fake site, never a user-provided hardware site."""
    resource = files(__package__).joinpath("fake_site.json")
    fleet = Fleet(site_from_dict(json.loads(resource.read_text())), strict=True).connect()
    result = {"data_origin": "fake"}
    try:
        body = fleet.bodies["demo"]
        control = body.control().acquire(["left", "right"])
        operations = [body.client.invoke(name, "move", {"target": .25, "duration_s": .05},
                                        control_id=control) for name in ("left", "right")]
        result["operations"] = [body.client.wait_operation(op["operation_id"], timeout_s=2)
                                for op in operations]
    finally:
        result["stop"] = fleet.shutdown(reason="fake demo completed", timeout_s=2)
        # No hardware is reachable from this function. Ensure a failing smoke test
        # still releases its in-memory fixture; real callers must retain unknowns.
        fleet.close()
    print(json.dumps(result, indent=2))
    return 0 if (all(op["state"] == "succeeded" for op in result["operations"])
                 and result["stop"]["status"] == "confirmed_idle") else 1


def validate(path):
    site = load_site(path)
    print(json.dumps({"site": site.name, "bodies": sorted(site.bodies),
                      "robots": sorted(site.robots), "connected": False}))
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("demo", help="Run the bundled in-memory fake; no hardware or LLM")
    check = sub.add_parser("validate", help="Validate site configuration without connecting")
    check.add_argument("site")
    args = parser.parse_args(argv)
    return demo() if args.command == "demo" else validate(args.site)


if __name__ == "__main__":
    raise SystemExit(main())
