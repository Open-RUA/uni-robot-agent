"""rua-host: the command dispatcher of the installed runtime archive; kits add their own commands."""

import argparse
import importlib
import json
import sys

from ..core.resources import runtime_root
from .config import load_host
from .kit import installed_kits, load_kit

BUILT_IN = {
    "body": __package__ + ".serve:main",
    "install-config": __package__ + ".install:main",
    "doctor": __package__ + ".doctor:main",
}


def commands():
    table = dict(BUILT_IN)
    for name in installed_kits():
        for command, target in load_kit(name).commands.items():
            if command in table or command in ("validate", "version"):
                raise ValueError(f"kit {name} redefines rua-host {command}")
            table[command] = target
    return table


def validate(argv):
    parser = argparse.ArgumentParser(prog="rua-host validate")
    parser.add_argument("--config", required=True)
    host = load_host(parser.parse_args(argv).config)
    print(json.dumps({"host_id": host["host_id"], "bodies": list(host["bodies"]),
                      "kits": sorted(host["kits"]), "hardware_accessed": False}))
    return 0


def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    table = commands()
    if not argv or argv[0] in ("-h", "--help"):
        print("rua-host: " + " | ".join(["validate", "version", *table]))
        return 0
    action = argv.pop(0)
    if action == "version":
        print((runtime_root() / "release.json").read_text())
        return 0
    if action == "validate":
        return validate(argv)
    if action not in table:
        raise ValueError(f"unknown host command {action!r}")
    module, _, function = table[action].partition(":")
    return getattr(importlib.import_module(module), function)(argv)


if __name__ == "__main__":
    raise SystemExit(main())
