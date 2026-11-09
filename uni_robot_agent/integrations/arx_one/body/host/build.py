"""Create reproducible CPython bytecode and native build inputs for the kits listed in body/runtime.json."""

from __future__ import annotations

import argparse
import gzip
import hashlib
import importlib
import importlib.util
import io
import json
import py_compile
import subprocess
import sys
import tarfile
import tempfile
import tomllib
import zipfile
from pathlib import Path

from .kit import load_kit

BODY_PACKAGE = __package__.rsplit(".", 1)[0]
BODY = BODY_PACKAGE.replace(".", "/")
SKIPPED_PARTS = {"__pycache__", "build", "install", "log"}


def _tree(source, archive_name, exclude=()):
    """Regular files under ``source`` mapped to archive names, without build outputs or bytecode."""
    if source.is_file():
        return {source: archive_name}
    if not source.is_dir():
        raise FileNotFoundError(f"native build input not found: {source}")
    return {path: f"{archive_name}/{path.relative_to(source).as_posix()}" for path in source.rglob("*")
            if path.is_file() and path.suffix != ".pyc" and path.name not in exclude
            and not SKIPPED_PARTS.intersection(path.relative_to(source).parts)}


def build(root, output, dependency_cache=None, *, target="linux-amd64", dependency_lock=None, manifest=None):
    if target not in {"linux-arm64", "linux-amd64"}:
        raise ValueError("unsupported hardware release target")
    if sys.version_info[:2] != (3, 12):
        raise RuntimeError(
            "Ubuntu 24.04 runtime artifacts must be built with CPython 3.12"
        )
    root = Path(root).resolve()
    version = tomllib.loads((root / "pyproject.toml").read_text())["project"]["version"]
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    hardware = root / BODY
    manifest_path = Path(manifest).resolve() if manifest else hardware / "runtime.json"
    manifest_bytes = manifest_path.read_bytes()
    runtime = json.loads(manifest_bytes)
    for field in ("name", "description", "kits"):
        if field not in runtime:
            raise ValueError(f"{manifest_path.name} needs {field}")
    if "\n" in runtime["description"] or not runtime["description"].strip():
        raise ValueError("the runtime description must be one nonempty line")
    kits = {name: load_kit(name) for name in runtime["kits"]}
    # Third-party native dependencies are optional: a runtime of pure-Python kits has none.
    lock_path = (Path(dependency_lock).resolve() if dependency_lock
                 else hardware / runtime["dependency_lock"] if runtime.get("dependency_lock") else None)
    lock_bytes = lock_path.read_bytes() if lock_path else b""
    lock = json.loads(lock_bytes) if lock_path else {}
    if lock_path and target not in (lock["release_targets"] if "release_targets" in lock else [lock["release_target"]]):
        raise ValueError("Dependency lock does not support the release target")
    files = {}
    # Native sources, models and SDK files sit beside the driver modules, so a
    # kit contributes its top-level Python files and the subpackages it lists.
    sources = [path for subtree in ("core", "host") for path in (hardware / subtree).rglob("*.py")]
    sources += [hardware / "__init__.py", hardware / "drivers/__init__.py"]
    native_files = {hardware / source: name for source, name in runtime.get("native_inputs", [])}
    release_fields = {}
    for name, kit in kits.items():
        kit_dir = hardware / "drivers" / name
        module = importlib.import_module(f"{BODY_PACKAGE}.drivers.{name}.kit")
        if not Path(module.__file__).resolve().is_relative_to(kit_dir):
            raise ValueError(f"kit {name} is imported from outside {root}; build from the imported checkout")
        sources += kit_dir.glob("*.py")
        for package in kit.build.packages:
            if not (kit_dir / package / "__init__.py").is_file():
                raise FileNotFoundError(f"kit {name} packages {package}, which is not a package")
            sources += (kit_dir / package).rglob("*.py")
        for source, archive_name in kit.build.native_inputs:
            for path, archived in _tree(kit_dir / source, f"{name}/{archive_name}", kit.build.exclude).items():
                if archived in native_files.values():
                    raise ValueError(f"two native inputs are archived as {archived}")
                native_files[path] = archived
        release_fields.update(kit.check_build(kit_dir, lock, target))
    for p in sources:
        if "__pycache__" not in p.parts:
            files[str(p.relative_to(root))] = p.read_bytes()
    # A hardware-only archive exposes the namespace, without importing the
    # application RobotSession registry and its agent/runtime dependencies.
    parts = BODY_PACKAGE.split(".")
    for depth in range(1, len(parts)):
        files["/".join(parts[:depth]) + "/__init__.py"] = b'"""Standalone hardware namespace."""\n'
    files["__main__.py"] = (
        f"from {__package__}.cli import main\nraise SystemExit(main())\n".encode()
    )
    # The Linux build runs the dependency tool as dependencies.py to stage the locked artifacts.
    tool = hardware / runtime["dependency_tool"] if runtime.get("dependency_tool") else None
    if tool:
        native_files[tool] = "dependencies.py"
    if lock_path:
        if "dependencies.lock.json" in native_files.values():
            raise ValueError("native_inputs must not include dependencies.lock.json; select the lock instead")
        native_files[lock_path] = "dependencies.lock.json"
    digest = hashlib.sha256()
    digest.update(target.encode() + b"\0" + manifest_bytes + b"\0")
    for name, value in sorted(files.items()):
        digest.update(name.encode() + b"\0" + value)
    for path, archive_name in sorted(native_files.items(), key=lambda item: item[1]):
        digest.update(archive_name.encode() + b"\0" + path.read_bytes())
    build_script = Path(__file__).with_name("build_runtime.sh")
    digest.update(build_script.name.encode() + b"\0" + build_script.read_bytes())
    identity = digest.hexdigest()
    build_id = version + "+" + identity[:12]
    with (
        tempfile.TemporaryDirectory() as temp,
        zipfile.ZipFile(
            output / "runtime.pyz", "w", compression=zipfile.ZIP_DEFLATED
        ) as archive,
    ):
        for name, value in sorted(files.items()):
            source = Path(temp) / name
            source.parent.mkdir(parents=True, exist_ok=True)
            source.write_bytes(value)
            bytecode = source.with_suffix(".pyc")
            py_compile.compile(
                str(source),
                cfile=str(bytecode),
                dfile="rua-runtime/" + name,
                doraise=True,
                invalidation_mode=py_compile.PycInvalidationMode.UNCHECKED_HASH,
            )
            info = zipfile.ZipInfo(name[:-3] + ".pyc", (2020, 1, 1, 0, 0, 0))
            info.compress_type = zipfile.ZIP_DEFLATED
            archive.writestr(info, bytecode.read_bytes())
    # Ignore host timestamps, ownership and the output filename in the archive.
    # Identical source bytes must produce identical artifacts on either host.
    native_archive = output / "native-sources.tar.gz"
    with (
        native_archive.open("wb") as raw,
        gzip.GzipFile(fileobj=raw, filename="", mode="wb", mtime=0) as compressed,
    ):
        with tarfile.open(fileobj=compressed, mode="w") as archive:
            for path, archive_name in sorted(native_files.items(), key=lambda item: item[1]):
                info = archive.gettarinfo(path, arcname=archive_name)
                if not info.isfile():
                    raise ValueError(f"native inputs must be regular files: {path}")
                info.uid = info.gid = info.mtime = 0
                info.uname = info.gname = ""
                info.mode = 0o755 if path.stat().st_mode & 0o111 else 0o644
                data = path.read_bytes()
                info.size = len(data)
                archive.addfile(info, io.BytesIO(data))
    installed_script = output / build_script.name
    installed_script.write_bytes(build_script.read_bytes())
    installed_script.chmod(0o755)
    manifest = {
        "build_id": build_id,
        "source_digest": identity,
        "python": f"{sys.version_info.major}.{sys.version_info.minor}",
        "bytecode_magic": importlib.util.MAGIC_NUMBER.hex(),
        "protocol_version": "2.0",
        "target": target,
        "runtime_sha256": hashlib.sha256(
            (output / "runtime.pyz").read_bytes()
        ).hexdigest(),
        "native_sources_sha256": hashlib.sha256(
            native_archive.read_bytes()
        ).hexdigest(),
        "build_script_sha256": hashlib.sha256(
            installed_script.read_bytes()
        ).hexdigest(),
        "dependency_lock_sha256": hashlib.sha256(lock_bytes).hexdigest() if lock_path else None,
        "package": runtime["name"],
        "description": runtime["description"],
        "kits": sorted(kits),
        "deb_depends": sorted({dep for kit in kits.values() for dep in kit.build.deb_depends}),
        **release_fields,
    }
    if dependency_cache is not None:
        if not (tool and lock_path):
            raise ValueError("a dependency cache needs dependency_tool and dependency_lock in the runtime manifest")
        bundle = output / "dependency-cache.tar.gz"
        subprocess.run(
            [
                sys.executable,
                str(tool),
                "bundle",
                "--lock",
                str(lock_path),
                "--cache",
                str(Path(dependency_cache).resolve()),
                "--output",
                str(bundle),
                "--target",
                target,
            ],
            check=True,
            capture_output=True,
        )
        manifest["dependency_cache_sha256"] = hashlib.sha256(
            bundle.read_bytes()
        ).hexdigest()
    (output / "release.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(json.dumps(manifest))
    return manifest


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--source", required=True)
    p.add_argument("--output", required=True)
    p.add_argument("--target", choices=["linux-arm64", "linux-amd64"], default="linux-amd64")
    p.add_argument("--dependency-lock", help="Explicit alternative lock; default release remains unchanged")
    p.add_argument("--manifest", help="Runtime manifest listing the kits to package; default body/runtime.json")
    p.add_argument(
        "--dependency-cache",
        help="Verified cache to include for a complete offline native build",
    )
    a = p.parse_args()
    build(a.source, a.output, a.dependency_cache, target=a.target, dependency_lock=a.dependency_lock,
          manifest=a.manifest)
