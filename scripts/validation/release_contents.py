"""Compare release archives with the public files in the source checkout.

Run after `python -m build`, which builds the wheel from the sdist by default.
This check uses only the Python standard library and never extracts archives.
"""
from __future__ import annotations

import argparse
from pathlib import Path, PurePosixPath
import tarfile
import zipfile


PUBLIC_DIRECTORIES = ('uni_robot_agent', 'docs', 'scripts', 'examples', 'assets', '.github')
EXCLUDED_INTEGRATIONS = {'robodojo_internal', 'efe', 'franka_robotiq',
                         'franka_sharpa', '_franka', 'dexterous_hand', 'dexterous_hand.py'}
ROOT_FILES = ('README.md', 'CHANGELOG.md', 'CONTRIBUTING.md', 'AGENTS.md',
              'CLAUDE.md', 'LICENSE', 'MANIFEST.in', 'pyproject.toml', 'pyrightconfig.json')
SOURCE_SUFFIXES = {'.py', '.md', '.toml', '.json', '.sh', '.txt', '.png', '.pdf',
                   '.html', '.css', '.js', '.ts', '.urdf', '.npz', '.npy', '.typed', '.yml',
                   # Native driver sources an integration builds on the robot host (C++, ROS 2).
                   '.yaml', '.cpp', '.hpp', '.h', '.msg', '.srv', '.xml'}
# Files without a suffix that travel with vendored native sources.
SOURCE_NAMES = {'UPSTREAM_LICENSE', 'LICENSE', 'NOTICE', 'SHA256SUMS'}


def public_files(root: Path) -> dict[str, bytes]:
    files = {name: (root / name).read_bytes() for name in ROOT_FILES}
    for directory in PUBLIC_DIRECTORIES:
        for path in sorted((root / directory).rglob('*')):
            if not path.is_file() or '__pycache__' in path.parts:
                continue
            if path.suffix in SOURCE_SUFFIXES or path.name in SOURCE_NAMES:
                files[path.relative_to(root).as_posix()] = path.read_bytes()
    return files


def read_sdist(path: Path) -> dict[str, bytes]:
    files = {}
    with tarfile.open(path, 'r:*') as archive:
        roots = {PurePosixPath(member.name).parts[0] for member in archive.getmembers()}
        if len(roots) != 1:
            raise ValueError('sdist must contain one distribution root')
        for member in archive.getmembers():
            if member.isdir():
                continue
            parts = PurePosixPath(member.name).parts
            if not member.isfile() or len(parts) < 2 or '..' in parts or parts[0] == '/':
                raise ValueError(f'Unsupported sdist member: {member.name}')
            name = '/'.join(parts[1:])
            if name in files:
                raise ValueError(f'Duplicate sdist member: {name}')
            with archive.extractfile(member) as stream:
                files[name] = stream.read()
    return files


def read_wheel(path: Path) -> dict[str, bytes]:
    with zipfile.ZipFile(path) as archive:
        names = [info.filename for info in archive.infolist() if not info.is_dir()]
        if len(names) != len(set(names)):
            raise ValueError('Duplicate wheel members')
        return {name: archive.read(name) for name in names}


def compare(label: str, expected: dict[str, bytes], actual: dict[str, bytes]) -> list[str]:
    errors = []
    for name, content in expected.items():
        if name not in actual:
            errors.append(f'{label}: missing {name}')
        elif actual[name] != content:
            errors.append(f'{label}: content differs for {name}')
    # Reject stale source files retained by a reused build directory or file manifest.
    for name in actual:
        if EXCLUDED_INTEGRATIONS.intersection(PurePosixPath(name).parts):
            errors.append(f'{label}: excluded integration content {name}')
        if name.split('/')[0] in PUBLIC_DIRECTORIES and name not in expected:
            errors.append(f'{label}: unexpected public file {name}')
    return errors


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source-root', type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument('--sdist', type=Path, required=True)
    parser.add_argument('--wheel', type=Path, required=True)
    args = parser.parse_args()
    try:
        expected = public_files(args.source_root.resolve())
        package = {name: content for name, content in expected.items()
                   if name.startswith('uni_robot_agent/')}
        errors = compare('sdist', expected, read_sdist(args.sdist))
        errors.extend(compare('wheel', package, read_wheel(args.wheel)))
        if errors:
            parser.exit(1, '\n'.join(errors) + '\n')
    except (OSError, ValueError, tarfile.TarError, zipfile.BadZipFile) as error:
        parser.exit(1, f'{error}\n')
    print(f'Release contents valid: {len(expected)} source files; {len(package)} package files.')


if __name__ == '__main__':
    main()
