#!/usr/bin/env python3
"""Generate/check the portable X5 model from the single ROS URDF source."""

import argparse
from pathlib import Path


def export(native_root: Path, *, check=False):
    source = native_root / "ros/src/arx_x5_controller/x5_2025.urdf"
    output = native_root / "models/arx_x5_2025/x5_2025.urdf"
    data = source.read_bytes()
    old = b"package://ultron_v1.1_aa/meshes/"
    if data.count(old) != 14:
        raise ValueError("Unexpected model mesh references; inspect the source URDF")
    generated = data.replace(old, b"./meshes/")
    if check:
        if not output.exists() or output.read_bytes() != generated:
            raise ValueError(
                "Portable URDF is stale; run python native/models/export.py"
            )
    else:
        output.write_bytes(generated)
    return output


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    print(export(Path(__file__).resolve().parents[1], check=args.check))
