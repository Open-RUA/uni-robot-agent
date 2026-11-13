"""Generate the independent native distribution from the same public source tree."""
from __future__ import annotations

from pathlib import Path
import re
import sys
import tomllib

import tomli_w

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'validation'))
from release_contents import public_files


def stage_runtime(root: Path, destination: Path) -> None:
    metadata = tomllib.loads((root / 'pyproject.toml').read_text())
    project = metadata['project']
    excluded = metadata['tool']['uni-robot-agent']['runtime']['exclude-dependencies']
    project['name'] = 'uni-robot-agent-runtime'
    project['readme'] = 'docs/pypi-runtime.md'
    project['description'] = 'Native service runtime for uni-robot-agent; install in a separate environment'
    project['dependencies'] = [requirement for requirement in project['dependencies']
                               if re.split(r'[<>=!~\[; ]', requirement)[0] not in excluded]
    project.pop('scripts')
    # Manual benchmark services need the same lightweight adapter dependencies.
    # Planner, development and other deployment extras remain excluded.
    project['optional-dependencies'] = {
        'benchmarks': project['optional-dependencies']['benchmarks'],
        'piper_x': project['optional-dependencies']['piper_x'],
    }
    # Both distributions own the same modules, but are never dependencies of one
    # another. Keeping the main wheel intact preserves upgrades from 0.1.0.
    destination.mkdir(parents=True, exist_ok=False)
    for name, content in public_files(root).items():
        target = destination / name
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(content)
    (destination / 'pyproject.toml').write_text(tomli_w.dumps(metadata))
