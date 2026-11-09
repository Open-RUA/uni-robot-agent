"""Create local two-arm AMD64 configuration copies; no network or hardware IO."""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
from pathlib import Path

from .limits import controller_profile, identity


PROFILE = "x5_2025_amd64_j4_90"
ORIGINAL = "x5_2025_amd64_type2"


def updated_configs(host, environment):
    host, environment = copy.deepcopy(host), copy.deepcopy(environment)
    profile = controller_profile(PROFILE)
    found = set()
    for body in host["bodies"].values():
        if body.get("kit") != "arx_x5":
            continue
        args = body["driver_args"]
        for device, arm in args["arms"].items():
            if arm.get("profile", args.get("profile")) != ORIGINAL:
                raise ValueError(f"Expected audited AMD64 type2 profile for {device}")
            arm["profile"] = PROFILE
            found.add(device)
        args["profile"] = PROFILE
    if found != {"arx.left_arm", "arx.right_arm"}:
        raise ValueError("Expected exactly the two commissioned ARX arms")
    if set(environment["arms"]) != {"left", "right"}:
        raise ValueError("Expected left and right environment arms")
    for side, arm in environment["arms"].items():
        if arm["device_id"] != f"arx.{side}_arm":
            raise ValueError("Environment device identity mismatch")
        metadata = arm["expected_metadata"]["identity"]
        if metadata.get("profile") != ORIGINAL:
            raise ValueError("Environment profile is not the audited baseline")
        original = controller_profile(ORIGINAL)
        if (metadata.get("sdk_sha256") != original.sdk_sha256
                or metadata.get("model_sha256") != original.model_sha256
                or arm["joint_lower"] != list(original.joint_lower)
                or arm["joint_upper"] != list(original.joint_upper)):
            raise ValueError("Environment bounds or SDK identity differ from baseline")
        arm["expected_metadata"]["identity"] = identity(PROFILE)
        arm["joint_lower"] = list(profile.joint_lower)
        arm["joint_upper"] = list(profile.joint_upper)
    return host, environment


def stage(host_path, environment_path, output):
    inputs = {"host.json": Path(host_path), "environment.json": Path(environment_path)}
    original = {name: path.read_bytes() for name, path in inputs.items()}
    host, env = updated_configs(*(json.loads(original[n]) for n in inputs))
    output = Path(output)
    output.mkdir(parents=True, exist_ok=False)
    (output / "original").mkdir()
    (output / "j4-90").mkdir()
    for name, updated in zip(inputs, (host, env)):
        (output / "original" / name).write_bytes(original[name])
        (output / "j4-90" / name).write_text(json.dumps(updated, indent=2) + "\n")
    manifest = {"profile": PROFILE, "deployed": False,
                "sources": {name: {"path": str(path.resolve()),
                            "sha256": hashlib.sha256(original[name]).hexdigest()}
                            for name, path in inputs.items()},
                "files": {str(p.relative_to(output)): hashlib.sha256(p.read_bytes()).hexdigest()
                          for p in output.glob("*/*.json")}}
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    return manifest


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", required=True)
    parser.add_argument("--environment", required=True)
    parser.add_argument("--output", required=True, help="New local directory; never overwrites inputs")
    args = parser.parse_args()
    stage(args.host, args.environment, args.output)
