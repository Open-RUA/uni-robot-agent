"""Offline summaries of saved run directories for the terminal."""
from __future__ import annotations

import csv
from dataclasses import dataclass, asdict
import io
import json
from pathlib import Path
import time
from typing import Any, Iterable, Sequence

from uni_robot_agent.artifacts.discovery import discover

LIVE_STATES = ('starting', 'deploying', 'running', 'finishing')
SUPERVISOR_FAILURE_STATES = (
    'deployment_failed', 'environment_failed', 'episode_timeout', 'cancelled',
)
STALE_AFTER_S = 15.0
STATUS_CHOICES = (
    'all', 'success', 'failed', 'finished', 'running', 'starting',
    'deploying', 'finishing', 'error', 'unknown', 'stale',
    'deployment_failed', 'environment_failed', 'episode_timeout', 'cancelled',
)
SORT_CHOICES = ('path', 'started', 'duration', 'task')

FIELDS = (
    'directory', 'name', 'task', 'instruction', 'integration', 'model',
    'agent_backend', 'status', 'task_success', 'accepted', 'steps',
    'max_steps', 'score', 'elapsed_s', 'started_at', 'finished_at', 'error',
)


@dataclass(frozen=True)
class RunSummary:
    """One saved run read from its JSON artifacts without a model or service."""

    directory: str
    name: str
    task: str | None
    instruction: str | None
    integration: str | None
    model: str | None
    agent_backend: str | None
    status: str
    task_success: bool | None
    accepted: bool | None
    steps: int | None
    max_steps: int | None
    score: float | None
    elapsed_s: float | None
    started_at: float | None
    finished_at: float | None
    error: str | None

    def as_dict(self) -> dict[str, Any]:
        return asdict(self)


def _read_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding='utf-8'))
    except (OSError, ValueError):
        return {}
    return value if isinstance(value, dict) else {}


def _budget(result: dict[str, Any], monitor: dict[str, Any], native: dict[str, Any]) -> int | None:
    budget = monitor.get('max_episode_steps')
    if budget is None:
        environment = (result.get('episode_identity') or {}).get('environment') or {}
        budget = environment.get('max_episode_steps')
    return native.get('step_limit') if budget is None else budget


def summarize(directory: str | Path) -> RunSummary:
    """Read one run directory into its terminal summary."""
    root = Path(directory).expanduser().resolve()
    result = _read_json(root / 'result.json')
    monitor = _read_json(root / 'monitor.json')
    supervisor = _read_json(root / 'supervisor.json')
    native = _read_json(root / 'native_outcome.json')

    monitor_state = monitor.get('state')
    heartbeat = monitor.get('heartbeat_at')
    started_at = monitor.get('started_at')
    finished_at = monitor.get('finished_at')
    if (monitor_state in LIVE_STATES and heartbeat is not None
            and time.time() - heartbeat > STALE_AFTER_S):
        monitor_state = 'stale'
        finished_at = heartbeat

    outcome = result.get('task_success') if result else None
    status = monitor_state or 'unknown'
    if result:
        status = 'success' if outcome is True else 'failed' if outcome is False else 'finished'
        if result.get('accepted') is False or (outcome is None and result.get('error')):
            status = 'error'
        if started_at is not None and finished_at is None and result.get('elapsed_s') is not None:
            finished_at = started_at + result['elapsed_s']
    elif supervisor.get('state') in SUPERVISOR_FAILURE_STATES:
        status = supervisor['state']

    steps = result.get('steps')
    if steps is None:
        steps = native.get('steps')
    name = root.name
    return RunSummary(
        directory=str(root),
        name=name,
        task=result.get('task') or monitor.get('task') or name,
        instruction=result.get('instruction') or monitor.get('instruction') or None,
        integration=result.get('integration') or monitor.get('integration') or None,
        model=result.get('model') or monitor.get('model') or None,
        agent_backend=result.get('agent_backend') or None,
        status=status,
        task_success=outcome,
        accepted=result.get('accepted') if result else None,
        steps=steps,
        max_steps=_budget(result, monitor, native),
        score=result.get('score', native.get('native_score')),
        elapsed_s=result.get('elapsed_s') if result else None,
        started_at=started_at,
        finished_at=finished_at,
        error=(result.get('error') or monitor.get('error')
               or supervisor.get('error') or None),
    )


def collect(paths: Iterable[str | Path]) -> list[RunSummary]:
    """Discover and summarize saved runs under the supplied paths."""
    roots = discover(paths)
    if not roots:
        raise ValueError('No result.json, trajectory.jsonl or monitor.json found under the specified directories')
    return [summarize(root) for root in roots]


def filter_rows(rows: Sequence[RunSummary], status: str = 'all') -> list[RunSummary]:
    return [row for row in rows if status == 'all' or row.status == status]


def _sort_with_missing_last(rows, value_of, reverse):
    present = [row for row in rows if value_of(row) is not None]
    present.sort(key=lambda row: (value_of(row), row.directory), reverse=reverse)
    return present + [row for row in rows if value_of(row) is None]


def sort_rows(rows: Sequence[RunSummary], sort: str = 'path', reverse: bool = False) -> list[RunSummary]:
    rows = list(rows)
    if sort == 'started':
        return _sort_with_missing_last(rows, lambda row: row.started_at, reverse)
    if sort == 'duration':
        return _sort_with_missing_last(rows, lambda row: row.elapsed_s, reverse)
    key = (lambda row: row.directory) if sort == 'path' else (
        lambda row: (row.task or '', row.directory))
    return sorted(rows, key=key, reverse=reverse)


def select_rows(rows: Sequence[RunSummary], *, status: str = 'all', sort: str = 'path',
                reverse: bool = False, limit: int | None = None) -> list[RunSummary]:
    selected = sort_rows(filter_rows(rows, status), sort, reverse)
    return selected[:limit] if limit is not None else selected


def _text(value) -> str:
    return '-' if value is None or value == '' else str(value)


def _truncate(value: str, width: int) -> str:
    return value if len(value) <= width else value[:width - 3] + '...'


def _steps(steps: int | None, max_steps: int | None) -> str | None:
    if steps is None:
        return None
    return f'{steps}/{max_steps}' if max_steps is not None else str(steps)


def _elapsed(seconds: float | None) -> str | None:
    if seconds is None:
        return None
    total = int(round(float(seconds)))
    if total < 60:
        return f'{float(seconds):.1f}s'
    minutes, remainder = divmod(total, 60)
    if minutes < 60:
        return f'{minutes}m{remainder:02d}s'
    hours, minutes = divmod(minutes, 60)
    return f'{hours}h{minutes:02d}m'


TABLE_HEADERS = ('RUN', 'STATE', 'TASK', 'MODEL', 'STEPS', 'ELAPSED', 'DIRECTORY')
TABLE_WIDTHS = (22, 19, 24, 18, 9, 9, 32)


def render_table(rows: Sequence[RunSummary]) -> str:
    def line(values):
        cells = [_truncate(_text(value), width).ljust(width)
                 for value, width in zip(values, TABLE_WIDTHS)]
        return '  '.join(cells).rstrip()

    lines = [line(TABLE_HEADERS)]
    for row in rows:
        lines.append(line((
            row.name, row.status, row.task, row.model,
            _steps(row.steps, row.max_steps), _elapsed(row.elapsed_s), row.directory,
        )))
    success = sum(row.status == 'success' for row in rows)
    failed = sum(row.status == 'failed' for row in rows)
    other = len(rows) - success - failed
    known = success + failed
    outcome = (f'{success}/{known} known outcomes, {100.0 * success / known:.1f}% success'
               if known else 'no known outcomes')
    lines.append('')
    lines.append(f'{len(rows)} runs: {success} success, {failed} failed, '
                 f'{other} other ({outcome})')
    return '\n'.join(lines)


def render_json(rows: Sequence[RunSummary]) -> str:
    return json.dumps([row.as_dict() for row in rows], ensure_ascii=False, indent=2)


def _csv_value(value):
    return '' if value is None else str(value)


def render_csv(rows: Sequence[RunSummary]) -> str:
    stream = io.StringIO()
    writer = csv.writer(stream, lineterminator='\n')
    writer.writerow(FIELDS)
    for row in rows:
        values = row.as_dict()
        writer.writerow([_csv_value(values[field]) for field in FIELDS])
    return stream.getvalue()


RENDERERS = {'table': render_table, 'json': render_json, 'csv': render_csv}


def print_runs(args) -> None:
    """Print the selected runs in the requested format to stdout."""
    rows = select_rows(
        collect(args.run_dir), status=args.status, sort=args.sort,
        reverse=args.reverse, limit=args.limit,
    )
    print(RENDERERS[args.format](rows), end='' if args.format == 'csv' else '\n')
