#!/usr/bin/env python3
"""Create a separate, hash-verified SDK source candidate. Never install or deploy."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def prepare(source, output):
    source, output = Path(source).resolve(), Path(output).resolve()
    if source == output or source in output.parents or output in source.parents:
        raise ValueError("SDK input and candidate must be separate, non-nested directories")
    if output.exists():
        raise FileExistsError("Candidate directory must not already exist")
    here = Path(__file__).resolve().parent
    manifest = json.loads((here / "clock-fix-source.json").read_text())
    # Validate everything before copying or altering any candidate files.
    for relative, hashes in manifest["files"].items():
        if sha(source / relative) != hashes["baseline_sha256"]:
            raise ValueError("SDK baseline mismatch: " + relative)
        if sha(here / Path(relative).name) != hashes["fixed_sha256"]:
            raise ValueError("Clock repair source checksum mismatch: " + relative)
    shutil.copytree(source, output, symlinks=True, ignore=shutil.ignore_patterns(".git", "build"))
    for relative in manifest["files"]:
        destination = output / relative
        # copytree preserves symlinks: never write through one into the input.
        if destination.is_symlink() or not destination.resolve().is_relative_to(output):
            raise ValueError("Candidate source must not point outside its directory")
        shutil.copyfile(here / Path(relative).name, destination)
    return output


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    print(prepare(args.source, args.output))
