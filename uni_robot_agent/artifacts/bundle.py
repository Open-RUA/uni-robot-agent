"""Package one saved run directory into a portable tar.gz bundle with a manifest."""
from __future__ import annotations

import gzip
import hashlib
import io
import json
import os
import stat
import tarfile
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from uuid import uuid4

MANIFEST_NAME = "ura_bundle_manifest.json"
FORMAT_VERSION = 1
RUN_MARKERS = ("result.json", "trajectory.jsonl", "monitor.json", "deployment.json")

# Planner/config state can contain account credentials and is never bundled.
AGENT_STATE_DIRECTORIES = frozenset({".claude", ".codex", ".agents"})
GENERATED_DIRECTORIES = frozenset({"home", "scratch", "tmp", "__pycache__", ".git"})
TRACE_DIRECTORIES = frozenset({"codex_responses", "claude_messages", "telemetry", "codex", "logs"})
MEDIA_DIRECTORIES = frozenset({"images", "videos", "monitor_media"})
MEDIA_SUFFIXES = frozenset(
    {".png", ".jpg", ".jpeg", ".webp", ".gif", ".bmp", ".mp4", ".webm", ".mov", ".avi"}
)
SENSITIVE_FILE_NAMES = frozenset({".env", ".netrc"})
SENSITIVE_FILE_SUFFIXES = frozenset({".pem", ".key", ".crt"})
SENSITIVE_FILE_PREFIXES = ("id_rsa", "credentials")


@dataclass(frozen=True)
class ArchiveResult:
    """Summary of one written bundle; the manifest is stored inside the archive."""

    output: Path
    source: Path
    file_count: int
    total_bytes: int
    skipped: tuple[dict[str, str], ...]
    files: tuple[dict[str, object], ...]
    include_media: bool
    include_traces: bool

    def summary(self) -> str:
        """Return the one-line CLI summary."""
        noun = "file" if self.file_count == 1 else "files"
        return (
            f"Archived {self.file_count} {noun} "
            f"({_format_bytes(self.total_bytes)}, {len(self.skipped)} skipped) "
            f"-> {self.output}"
        )


def archive_run(
    run_dir: str | Path,
    output: str | Path,
    *,
    include_media: bool = True,
    include_traces: bool = True,
    force: bool = False,
) -> ArchiveResult:
    """Write one saved run directory into an atomic ``.tar.gz`` bundle.

    The bundle keeps each file's path relative to the run directory and adds
    ``ura_bundle_manifest.json`` at its root. Symlinks, credential files and
    generated agent state are skipped and recorded in the manifest. The output
    must not be inside the run directory and its parent must already exist.
    """
    source = _resolve_run_dir(run_dir)
    destination = _resolve_output(output, source, force=force)
    collected, skipped = _collect_files(
        source,
        destination,
        include_media=bool(include_media),
        include_traces=bool(include_traces),
    )
    files, total_bytes = _write_bundle(
        source,
        destination,
        collected,
        skipped,
        include_media=bool(include_media),
        include_traces=bool(include_traces),
    )
    return ArchiveResult(
        output=destination,
        source=source,
        file_count=len(files),
        total_bytes=total_bytes,
        skipped=tuple(dict(item) for item in skipped),
        files=tuple(dict(item) for item in files),
        include_media=bool(include_media),
        include_traces=bool(include_traces),
    )


def _resolve_run_dir(value: str | Path) -> Path:
    source = Path(value).expanduser().resolve()
    if not source.exists():
        raise ValueError(f"Run directory does not exist: {source}")
    if not source.is_dir():
        raise ValueError(f"Run directory is not a directory: {source}")
    if not any((source / marker).is_file() for marker in RUN_MARKERS):
        raise ValueError(
            "Run directory has no run marker "
            f"({', '.join(RUN_MARKERS)}): {source}"
        )
    manifest_path = source / MANIFEST_NAME
    if manifest_path.exists() or manifest_path.is_symlink():
        raise ValueError(
            f"Run directory already contains {MANIFEST_NAME}; refusing to overwrite it"
        )
    return source


def _resolve_output(value: str | Path, source: Path, *, force: bool) -> Path:
    output = Path(value).expanduser()
    if not output.is_absolute():
        output = Path.cwd() / output
    parent = output.parent.resolve()
    if not parent.exists():
        raise ValueError(f"Output parent directory does not exist: {parent}")
    if not parent.is_dir():
        raise ValueError(f"Output parent is not a directory: {parent}")
    destination = parent / output.name
    if destination == source or destination.is_relative_to(source):
        raise ValueError(f"Output must not be inside the run directory: {destination}")
    if destination.exists() or destination.is_symlink():
        if destination.is_dir():
            raise ValueError(f"Output path is a directory: {destination}")
        if not force:
            raise ValueError(
                f"Output already exists: {destination} (pass --force to replace it)"
            )
    return destination


def _collect_files(
    source: Path,
    destination: Path,
    *,
    include_media: bool,
    include_traces: bool,
) -> tuple[list[tuple[str, Path]], list[dict[str, str]]]:
    """Return (relative path, absolute path) pairs and recorded exclusions."""
    collected: list[tuple[str, Path]] = []
    skipped: list[dict[str, str]] = []

    def exclude(relative: Path, reason: str) -> None:
        skipped.append({"path": relative.as_posix(), "reason": reason})

    def walk(relative_dir: Path) -> None:
        absolute_dir = source / relative_dir
        try:
            with os.scandir(absolute_dir) as iterator:
                entries = sorted(iterator, key=lambda entry: entry.name)
        except OSError as error:
            raise OSError(f"Cannot read run directory {absolute_dir}: {error}") from error
        for entry in entries:
            relative = relative_dir / entry.name
            if entry.is_symlink():
                exclude(relative, "symlink")
                continue
            try:
                is_directory = entry.is_dir(follow_symlinks=False)
                is_file = entry.is_file(follow_symlinks=False)
            except OSError as error:
                raise OSError(f"Cannot inspect run entry {source / relative}: {error}") from error
            if is_directory:
                reason = _directory_exclusion(
                    entry.name, include_media=include_media, include_traces=include_traces
                )
                if reason is not None:
                    exclude(relative, reason)
                else:
                    walk(relative)
                continue
            if not is_file:
                exclude(relative, "not a regular file")
                continue
            reason = _file_exclusion(entry.name, include_media=include_media)
            if reason is not None:
                exclude(relative, reason)
                continue
            absolute = source / relative
            try:
                resolved = absolute.resolve()
            except OSError as error:
                raise OSError(f"Cannot resolve run file {absolute}: {error}") from error
            if not resolved.is_relative_to(source):
                exclude(relative, "outside run directory")
                continue
            if resolved == destination:
                exclude(relative, "output bundle")
                continue
            collected.append((relative.as_posix(), absolute))

    walk(Path())
    collected.sort(key=lambda item: item[0])
    skipped.sort(key=lambda item: (item["path"], item["reason"]))
    return collected, skipped


def _directory_exclusion(name: str, *, include_media: bool, include_traces: bool) -> str | None:
    if name in AGENT_STATE_DIRECTORIES:
        return "agent state directory"
    if name in GENERATED_DIRECTORIES:
        return "generated directory"
    if not include_media and name in MEDIA_DIRECTORIES:
        return "media directory excluded (include_media=False)"
    if not include_traces and name in TRACE_DIRECTORIES:
        return "trace directory excluded (include_traces=False)"
    return None


def _file_exclusion(name: str, *, include_media: bool) -> str | None:
    lowered = name.lower()
    if name in SENSITIVE_FILE_NAMES or lowered.startswith(".env.") or lowered == ".env":
        return "credential file"
    if _suffix(lowered) in SENSITIVE_FILE_SUFFIXES:
        return "credential file"
    if lowered.startswith(SENSITIVE_FILE_PREFIXES):
        return "credential file"
    if not include_media and _suffix(lowered) in MEDIA_SUFFIXES:
        return "media file excluded (include_media=False)"
    return None


def _suffix(name: str) -> str:
    return Path(name).suffix.lower()


def _write_bundle(
    source: Path,
    destination: Path,
    collected: list[tuple[str, Path]],
    skipped: list[dict[str, str]],
    *,
    include_media: bool,
    include_traces: bool,
) -> tuple[list[dict[str, object]], int]:
    temporary = destination.parent / f".{destination.name}.{uuid4().hex}.tmp"
    try:
        with temporary.open("wb") as raw:
            # Fixed gzip mtime keeps archives reproducible apart from file contents.
            with gzip.GzipFile(fileobj=raw, mode="wb", filename="", mtime=0) as compressed:
                with tarfile.open(fileobj=compressed, mode="w") as archive:
                    files = []
                    for name, absolute in collected:
                        size, digest = _add_regular_file(archive, absolute, name)
                        files.append({"path": name, "size": size, "sha256": digest})
                    manifest = _build_manifest(
                        source,
                        files,
                        skipped,
                        include_media=include_media,
                        include_traces=include_traces,
                    )
                    _add_manifest(archive, manifest)
            raw.flush()
            os.fsync(raw.fileno())
        os.replace(temporary, destination)
    except BaseException:
        temporary.unlink(missing_ok=True)
        raise
    return files, int(manifest["total_bytes"])


class _HashingReader:
    """Stream file bytes into the tar writer while hashing exactly what is stored."""

    def __init__(self, stream) -> None:
        self._stream = stream
        self._hash = hashlib.sha256()
        self.total = 0

    def read(self, size: int = -1) -> bytes:
        data = self._stream.read(size)
        if data:
            self._hash.update(data)
            self.total += len(data)
        return data

    def hexdigest(self) -> str:
        return self._hash.hexdigest()


def _add_regular_file(archive: tarfile.TarFile, path: Path, name: str) -> tuple[int, str]:
    flags = os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0)
    try:
        descriptor = os.open(path, flags)
    except OSError as error:
        raise OSError(f"Cannot open run file {path}: {error}") from error
    with os.fdopen(descriptor, "rb") as stream:
        stat_result = os.fstat(stream.fileno())
        if not stat.S_ISREG(stat_result.st_mode):
            raise OSError(f"Run file is not a regular file: {path}")
        info = _tar_info(name, stat_result.st_size)
        reader = _HashingReader(stream)
        archive.addfile(info, reader)
        if reader.total != info.size:
            raise OSError(f"Run file changed while archiving: {path}")
    return reader.total, reader.hexdigest()


def _add_manifest(archive: tarfile.TarFile, manifest: dict[str, object]) -> None:
    payload = (json.dumps(manifest, ensure_ascii=False, indent=2) + "\n").encode("utf-8")
    info = _tar_info(MANIFEST_NAME, len(payload))
    archive.addfile(info, io.BytesIO(payload))


def _tar_info(name: str, size: int) -> tarfile.TarInfo:
    info = tarfile.TarInfo(name=name)
    info.size = size
    info.mtime = 0
    info.mode = 0o644
    info.uid = 0
    info.gid = 0
    info.uname = ""
    info.gname = ""
    return info


def _build_manifest(
    source: Path,
    files: list[dict[str, object]],
    skipped: list[dict[str, str]],
    *,
    include_media: bool,
    include_traces: bool,
) -> dict[str, object]:
    from uni_robot_agent import __version__

    return {
        "format_version": FORMAT_VERSION,
        "created_at": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "package_version": __version__,
        "source": str(source),
        "options": {
            "include_media": include_media,
            "include_traces": include_traces,
        },
        "files": files,
        "skipped": skipped,
        "file_count": len(files),
        "total_bytes": sum(int(item["size"]) for item in files),
    }


def _format_bytes(value: int) -> str:
    size = float(max(0, value))
    for unit in ("B", "KiB", "MiB"):
        if size < 1024:
            return f"{int(size)} B" if unit == "B" else f"{size:.1f} {unit}"
        size /= 1024
    return f"{size:.1f} GiB"
