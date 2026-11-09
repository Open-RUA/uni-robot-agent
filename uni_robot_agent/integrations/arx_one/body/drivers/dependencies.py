#!/usr/bin/env python3
"""Fetch, verify and stage locked native dependencies; never accesses hardware."""

from __future__ import annotations

import argparse
import copy
import gzip
import hashlib
import json
import math
import shutil
import struct
import subprocess
import tarfile
import tempfile
import urllib.request
from pathlib import Path, PurePosixPath


def apply_arx_j4_90(data: bytes) -> bytes:
    """Version-locked local SDK variant; change only the two J4 data constants.

    Both clipping loops keep running. The input vendor artifact remains intact
    in the hash-addressed cache, and the original release is retained for rollback.
    This is an RUA modification, not a manufacturer SDK release.
    """
    original = "fb4e905d523f16d331b46089f942d5cff47f53a85acf774519dfa1d17909234a"
    expected = "3742728fa1ef56eef5eaf5b3a552e0b8e3c6944ba1cd1b937fdb88e9fddc2265"
    if hashlib.sha256(data).hexdigest() != original:
        raise ValueError("J4 patch requires the exact audited official ARM64 SDK")
    output = bytearray(data)
    for offset, old, new in (
        (0x1A76C8, -1.29, -math.radians(90)),
        (0x1A7700, 1.29, math.radians(90)),
    ):
        if data[offset:offset + 8] != struct.pack("<d", old):
            raise ValueError("J4 patch original constant mismatch")
        output[offset:offset + 8] = struct.pack("<d", new)
    if hashlib.sha256(output).hexdigest() != expected:
        raise ValueError("J4 patch output checksum mismatch")
    return bytes(output)


def digest(path):
    h = hashlib.sha256()
    with Path(path).open("rb") as stream:
        while block := stream.read(1024 * 1024):
            h.update(block)
    return h.hexdigest()


def apply_arx_amd64_j4_90(data: bytes) -> bytes:
    """Patch only the audited AMD64 J4 constants, never load the controller.

    Constructor loads these doubles into lower[3]/upper[3]. Both
    stateEndControl and statePositionControl use the same clipping arrays.
    This is a local variant, not a vendor release; original cache is immutable.
    """
    original = "d07d53d14da17f0a1db2753e3339f9e4c055dad72df613c811b9692c4bede85c"
    expected = "ec4a7e7f2528797c6e9fcf1b30e773165be81b8d8130ed28c00510fd651e67a7"
    if hashlib.sha256(data).hexdigest() != original:
        raise ValueError("J4 patch requires the exact audited official AMD64 SDK")
    output = bytearray(data)
    for offset, old, new in (
        (0x1BF328, -1.29, -math.pi / 2),
        (0x1BF358, 1.29, math.pi / 2),
    ):
        if data[offset:offset + 8] != struct.pack("<d", old):
            raise ValueError("AMD64 J4 patch original constant mismatch")
        output[offset:offset + 8] = struct.pack("<d", new)
    if hashlib.sha256(output).hexdigest() != expected:
        raise ValueError("AMD64 J4 patch output checksum mismatch")
    return bytes(output)


def selected(lock, target):
    if target not in lock.get("release_targets", [lock["release_target"]]):
        raise ValueError(
            "Only the locked release target has a complete validated SDK/toolchain"
        )
    return [
        a
        for d in lock["dependencies"].values()
        for a in d["artifacts"]
        if a["target"] in ("all", target)
    ]


def fetch(lock_path, cache, *, offline=False, imports=(), target="linux-arm64"):
    lock = json.loads(Path(lock_path).read_text())
    if lock.get("schema_version") != 1:
        raise ValueError("Unsupported native dependency lock")
    cache = Path(cache)
    cache.mkdir(parents=True, exist_ok=True)
    supplied = {Path(p).name: Path(p) for p in imports}
    for artifact in selected(lock, target):
        expected = artifact["sha256"]
        if len(expected) != 64 or any(c not in "0123456789abcdef" for c in expected):
            raise ValueError("Invalid SHA-256 in dependency lock")
        blob = cache / expected
        if blob.exists():
            if digest(blob) != expected:
                raise ValueError(
                    f"Corrupt cached dependency {expected}; refusing to use it"
                )
            continue
        with tempfile.NamedTemporaryFile(dir=cache, delete=False) as stream:
            temporary = Path(stream.name)
        try:
            imported = supplied.get(artifact.get("import_name"))
            if imported is not None:
                shutil.copyfile(imported, temporary)
            elif offline or not artifact.get("url"):
                raise FileNotFoundError(
                    f"Missing dependency {artifact.get('import_name', expected)}; supply the offline cache or --import-artifact"
                )
            else:
                with (
                    urllib.request.urlopen(artifact["url"], timeout=60) as source,
                    temporary.open("wb") as destination,
                ):
                    shutil.copyfileobj(source, destination)
            if digest(temporary) != expected:
                raise ValueError(f"Dependency checksum mismatch: {artifact.get('url')}")
            temporary.replace(blob)
        finally:
            temporary.unlink(missing_ok=True)
    return lock


def bundle(lock_path, cache, output, *, target="linux-arm64"):
    lock = fetch(lock_path, cache, offline=True, target=target)
    with (
        Path(output).open("wb") as raw,
        gzip.GzipFile(fileobj=raw, filename="", mode="wb", mtime=0) as gz,
    ):
        with tarfile.open(fileobj=gz, mode="w") as archive:
            for name in sorted({a["sha256"] for a in selected(lock, target)}):
                file = Path(cache) / name
                info = tarfile.TarInfo(name)
                info.size = file.stat().st_size
                info.mode = 0o644
                with file.open("rb") as stream:
                    archive.addfile(info, stream)


def unpack(blob, destination, prefix=None):
    with tarfile.open(blob) as archive:
        members = []
        for original in archive.getmembers():
            member = copy.copy(original)
            path = PurePosixPath(member.name)
            if path.is_absolute() or ".." in path.parts:
                raise ValueError("Unsafe archive member")
            if prefix:
                if not path.parts or path.parts[0] != prefix:
                    raise ValueError("Archive does not match the locked root")
                member.name = str(PurePosixPath(*path.parts[1:]))
            members.append(member)
        archive.extractall(destination, members=members, filter="data")


def prepare(lock_path, cache, output, *, target="linux-arm64"):
    lock = fetch(lock_path, cache, offline=True, target=target)
    output = Path(output)
    if output.exists() and any(output.iterdir()):
        raise ValueError("Dependency staging directory must be empty")
    output.mkdir(parents=True, exist_ok=True)
    patched_files = {}
    for artifact in selected(lock, target):
        destination = output / artifact["destination"]
        if not destination.resolve().is_relative_to(output.resolve()):
            raise ValueError("Dependency destination escapes staging directory")
        blob = Path(cache) / artifact["sha256"]
        if artifact["format"] == "file":
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(blob, destination)
        elif artifact["format"] == "tar":
            destination.mkdir(parents=True, exist_ok=True)
            unpack(blob, destination, artifact.get("strip_prefix"))
        elif artifact["format"] == "deb":
            subprocess.run(
                ["dpkg-deb", "--extract", str(blob), str(destination)], check=True
            )
        else:
            raise ValueError("Unsupported dependency archive")
        if artifact.get("transform"):
            transforms = {
                ("rua_arx_arm64_j4_90_v1", "linux-arm64", "arx/arx_x5_src/libarx_x5_src-arm64.so"): apply_arx_j4_90,
                ("rua_arx_amd64_j4_90_v1", "linux-amd64", "arx/arx_x5_src/libarx_x5_src.so"): apply_arx_amd64_j4_90,
            }
            transform = transforms.get((artifact["transform"], target, artifact["destination"]))
            if transform is None or artifact["format"] != "file":
                raise ValueError("Unsupported native dependency transformation")
            transformed = transform(destination.read_bytes())
            if hashlib.sha256(transformed).hexdigest() != artifact.get("output_sha256"):
                raise ValueError("Transformed SDK differs from dependency lock")
            destination.write_bytes(transformed)
            patched_files[artifact["destination"]] = artifact["output_sha256"]
    if "runtime_files" in lock:
        checks = dict(lock["runtime_files"][target])
        if not checks:
            raise ValueError("Target requires verified runtime file hashes")
    else:
        checks = {
            "can-utils/usr/bin/slcand": lock["dependencies"]["can_utils"][
                "installed_slcand_sha256"
            ],
            "realsense/lib/librealsense2.so.2.58.3": lock["dependencies"]["librealsense"][
                "shared_library_sha256"
            ],
        }
    checks.update(patched_files)
    for filename, expected in checks.items():
        if digest(output / filename) != expected:
            raise ValueError(
                f"Installed dependency differs from verified baseline: {filename}"
            )
    manifest = {
        "target": target,
        "lock_sha256": digest(lock_path),
        "versions": {
            n: {**{k: v for k, v in d.items() if k not in ("artifacts", "target_overrides")},
                **d.get("target_overrides", {}).get(target, {})}
            for n, d in lock["dependencies"].items()
        },
        "artifact_sha256": [a["sha256"] for a in selected(lock, target)],
        "verified_runtime_files": checks,
    }
    (output / "dependencies.resolved.json").write_text(
        json.dumps(manifest, indent=2) + "\n"
    )
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=["fetch", "bundle", "prepare"])
    parser.add_argument(
        "--lock", type=Path, default=Path(__file__).with_name("dependencies.lock.json")
    )
    parser.add_argument("--cache", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--offline", action="store_true")
    parser.add_argument("--target", default="linux-arm64")
    parser.add_argument("--import-artifact", type=Path, action="append", default=[])
    args = parser.parse_args()
    if args.command == "fetch":
        fetch(args.lock, args.cache, offline=args.offline, imports=args.import_artifact,
              target=args.target)
    else:
        if not args.output:
            parser.error("--output is required for bundle/prepare")
        (bundle if args.command == "bundle" else prepare)(
            args.lock, args.cache, args.output, target=args.target
        )
    print(f"Native dependencies {args.command}: verified")


if __name__ == "__main__":
    main()
