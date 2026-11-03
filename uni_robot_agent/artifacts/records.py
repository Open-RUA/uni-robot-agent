"""Structured readers for saved results and trajectory artifacts."""
from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Any, Iterator

from uni_robot_agent.artifacts.run import read_result_json


@dataclass(frozen=True)
class RunResult:
    """Finalized episode result, including persistence failures independently of success."""

    session_dir: Path
    record: dict[str, Any]
    exit_code: int

    @property
    def task_success(self) -> bool | None:
        return self.record["task_success"]

    @property
    def agent_finished(self) -> bool:
        return self.record["agent_finished"]

    @property
    def accepted(self) -> bool:
        return self.record["accepted"]

    @property
    def retryable(self) -> bool:
        return self.record["retryable"]

    @property
    def failure_category(self) -> str | None:
        return self.record["failure_category"]

    @property
    def storage_errors(self) -> dict[str, str]:
        return dict(self.record.get("storage_errors", {}))


@dataclass(frozen=True)
class RunRecord:
    """Read a completed run without loading agents or connecting to an environment."""

    session_dir: Path

    @classmethod
    def open(cls, directory: str | Path) -> RunRecord:
        path = Path(directory).expanduser().resolve()
        read_result_json(path / "result.json")
        return cls(path)

    @property
    def result(self) -> dict[str, Any]:
        return read_result_json(self.session_dir / "result.json")

    def events(self) -> Iterator[dict[str, Any]]:
        with (self.session_dir / "trajectory.jsonl").open(encoding="utf-8") as stream:
            for line in stream:
                if line.strip():
                    yield json.loads(line)

    @property
    def measurements(self) -> dict[str, Any] | None:
        """Reward-independent facts; older runs may not have this artifact."""
        path = self.session_dir / "measurements.json"
        return json.loads(path.read_text(encoding="utf-8")) if path.is_file() else None

    def artifact(self, relative_path: str) -> Path:
        path = (self.session_dir / relative_path).resolve()
        if not path.is_relative_to(self.session_dir):
            raise ValueError("Artifact path must remain inside the run directory")
        return path
