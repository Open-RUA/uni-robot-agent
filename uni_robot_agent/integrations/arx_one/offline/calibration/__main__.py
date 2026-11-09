"""Explicit offline calibration CLI; outputs are created without overwriting."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

from ...perception.extrinsics import fit_calibration, load_calibration


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    inspect = commands.add_parser("inspect", help="Validate and summarize a record without hardware")
    inspect.add_argument("record", type=Path)
    for name in ("fit-points", "solve-hand-eye", "solve-eye-in-hand", "verify-hand-eye"):
        command = commands.add_parser(name)
        command.add_argument("--input", required=True, type=Path)
        command.add_argument("--output", required=True, type=Path)
        if name == "verify-hand-eye":
            command.add_argument("--record", required=True, type=Path)
    args = parser.parse_args(argv)
    try:
        if args.command == "inspect":
            loaded = load_calibration(args.record)
            record = loaded.record
            print(json.dumps({"status": record["status"], "sha256": loaded.sha256,
                              "source_frame": record["source_frame"], "target_frame": record["target_frame"],
                              "validation_max_m": record.get("validation_max_m"),
                              "motion_ready": False}, indent=2))
            return 0
        raw = args.input.read_bytes()
        data = json.loads(raw)
        if args.command == "fit-points":
            result = fit_calibration(data)
        elif args.command == "solve-hand-eye":
            from .hand_eye import solve_eye_to_hand
            result = solve_eye_to_hand(data)
        elif args.command == "solve-eye-in-hand":
            from .eye_in_hand import solve_eye_in_hand
            result = solve_eye_in_hand(data)
        else:
            from .workflows import verify_frozen_pair
            loaded = load_calibration(args.record)
            result = verify_frozen_pair(loaded.record, data)
            result["record_sha256"] = loaded.sha256
        result["input_sha256"] = hashlib.sha256(raw).hexdigest()
        rendered = json.dumps(result, indent=2, allow_nan=False) + "\n"
        with args.output.open("x") as output:
            output.write(rendered)
        print(str(args.output))
        return 0 if result.get("passed", result.get("status") == "validated") else 2
    except (ValueError, KeyError, TypeError, OSError, ImportError) as exc:
        parser.exit(2, f"calibration: {exc}\n")


if __name__ == "__main__":
    raise SystemExit(main())
