"""Discover saved run directories from explicit paths without a runtime."""
from __future__ import annotations

import os
from pathlib import Path

# A directly supplied directory is a run when it holds any of these markers.
RUN_MARKERS = ('result.json', 'trajectory.jsonl', 'monitor.json', 'deployment.json')
# Recursive discovery only treats episode artifacts as run markers.
NESTED_RUN_MARKERS = {'result.json', 'trajectory.jsonl', 'monitor.json'}
# Image/model caches are leaves, not potential run directories.
EXCLUDED_DIRECTORIES = {
    'images', 'artifacts', 'videos', 'monitor_media', '.codex',
    'codex_responses', 'telemetry', 'scratch', 'home', 'tmp',
}


def discover(paths):
    """Return the unique run directories named by paths or found beneath them."""
    roots = []
    for raw in paths:
        path = Path(raw).expanduser().resolve()
        if not path.is_dir():
            raise ValueError(f'Run directory does not exist: {path}')
        if any((path / f).is_file() for f in RUN_MARKERS):
            roots.append(path)
        else:
            found = set()
            for directory, children, files in os.walk(path, followlinks=False):
                children[:] = [n for n in children if n not in EXCLUDED_DIRECTORIES]
                if set(files) & NESTED_RUN_MARKERS:
                    found.add(Path(directory))
                    children[:] = []
            roots.extend(sorted(found))
    return list(dict.fromkeys(roots))
