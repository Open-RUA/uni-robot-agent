"""Serve Uni's environment protocol in the Harbor agent container."""
import argparse
from pathlib import Path

from uni_robot_agent.integrations.simulation import serve_episode

# Verified appends raw-client instructions to every task; the Direct tools replace them.
# A launcher may already have removed them.
OPERATING_HEADING = "\n## Operating this environment"


def task_text(text):
    task = text.partition(OPERATING_HEADING)[0].strip()
    if not task:
        raise ValueError("Instruction file has no task statement")
    return task


def make_adapter(client, instruction, episode_id, frame_table=None):
    """Pick the adapter by the package's declared action layout; an unregistered layout is refused, never guessed."""
    from .adapter import UnsupportedLayout, VerifiedAdapter

    names = client.describe()["action"]["names"]
    if any(str(name).startswith(("left", "right")) for name in names):
        from .dual_adapter import DualVerifiedAdapter, FrameTable

        if frame_table is None:
            raise UnsupportedLayout("Dual-arm packages need a frame table; none was supplied")
        return DualVerifiedAdapter(client, instruction, episode_id, FrameTable.load(frame_table))
    return VerifiedAdapter(client, instruction, episode_id)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--instruction-file", type=Path, required=True)
    parser.add_argument("--connection", default="/opt/roboenv/connection.json")
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=18760)
    parser.add_argument("--frame-table", type=Path, help="Dual-arm arm-base table built by the harness")
    args = parser.parse_args()

    from roboenv_client import Client

    instruction = task_text(args.instruction_file.read_text())
    client = Client(args.connection)
    args.run_id = client.observe()["episode_id"]
    serve_episode(lambda: make_adapter(client, instruction, args.run_id, args.frame_table), args, "robouniverse")


if __name__ == "__main__":
    main()
