"""Single physical episode served through the existing EnvironmentService routes."""
import argparse
from contextlib import asynccontextmanager
from pathlib import Path
import threading

from fastapi import FastAPI
import uvicorn

from uni_robot_agent.artifacts.run import write_json_atomic
from uni_robot_agent.artifacts.store import ArtifactStore
from uni_robot_agent.services.environment.contracts import EpisodeInfo
from uni_robot_agent.services.environment.routes import build_router
from uni_robot_agent.services.environment.service import EnvironmentService
from uni_robot_agent.services.environment.snapshots import SnapshotStore
from ..config import Config
from ..perception.completion.client import create_depth_backend
from .adapter import Adapter


def build_service(config, output_dir, *, episode_id, instruction, depth_backend=None):
    output_dir = Path(output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    adapter = Adapter(config, output_dir, episode_id=episode_id, depth_backend=depth_backend)
    try:
        info = EpisodeInfo(episode_id=episode_id, task=instruction, integration="arx_one",
            environment=adapter.description, metadata={"instruction": instruction, "skill_namespace": "arx_one"})
        artifacts = ArtifactStore(output_dir)
        service = EnvironmentService(adapter, info, artifacts, SnapshotStore(artifacts))
        service.initialize()
        return service
    except BaseException:
        adapter.close()
        raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--instruction", required=True)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=18760)
    parser.add_argument("--depth-config", type=Path, help="Enable depth completion on captured scenes")
    args = parser.parse_args()
    config = Config.load(args.config)  # before starting a depth worker, so a bad config leaks nothing
    depth_backend = (create_depth_backend(args.depth_config, args.output_dir / "depth-worker")
                     if args.depth_config else None)
    try:
        service = build_service(config, args.output_dir, episode_id=args.run_id,
                                instruction=args.instruction, depth_backend=depth_backend)
    except BaseException:
        if depth_backend is not None:
            depth_backend.close()
        raise
    serve(service, args.output_dir, host=args.host, port=args.port)


def serve(service, output_dir, *, host="127.0.0.1", port=18760):
    """Serve one episode; the stop evidence is written before the process can exit."""
    path = Path(output_dir) / "environment-outcome.json"

    def record(reason):
        outcome = service.backend.finish(reason)
        write_json_atomic(path, outcome, allow_nan=False)
        return outcome

    @asynccontextmanager
    async def lifespan(app):
        try:
            yield
        finally:
            # uvicorn re-raises SIGTERM/SIGINT once shutdown completes, so code after
            # run() never executes on a signal. Stop and record here instead.
            record("server_shutdown")
            service.close()

    class Server(uvicorn.Server):
        def handle_exit(self, sig, frame):
            # Start the physical stop at once; the HTTP drain may wait on an action.
            threading.Thread(target=service.backend.finish, args=("operator_cancelled",), daemon=True).start()
            super().handle_exit(sig, frame)

    app = FastAPI(lifespan=lifespan)
    app.include_router(build_router(service))
    try:
        Server(uvicorn.Config(app, host=host, port=port)).run()
    finally:
        # Startup failures and normal returns: finish is idempotent and keeps its first outcome.
        record("server_shutdown")
        service.backend.close()


if __name__ == "__main__":
    main()
