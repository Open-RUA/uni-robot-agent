"""Serve one pinned physical episode with the shared EnvironmentService."""
from contextlib import asynccontextmanager
from pathlib import Path
import threading

from uni_robot_agent.artifacts.run import write_json_atomic


def build_service(installation, output_dir, *, episode_id, instruction, execute=False, max_episode_steps=None):
    from uni_robot_agent.artifacts.store import ArtifactStore
    from uni_robot_agent.integrations.piper_x.environment.adapter import Adapter
    from uni_robot_agent.services.environment.contracts import EpisodeInfo
    from uni_robot_agent.services.environment.service import EnvironmentService
    output_dir = Path(output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=False)
    audit = installation.audit()
    from ..version import fingerprint
    audit["software"] = fingerprint()
    write_json_atomic(output_dir / "deployment.redacted.json", audit)
    write_json_atomic(output_dir / "configuration.redacted.json", installation.redacted_configuration())
    config = installation.environment(execute=execute)
    if max_episode_steps is not None:
        if not 0 < max_episode_steps <= config.max_episode_steps:
            raise ValueError("Run step budget exceeds station cap")
        config.max_episode_steps = max_episode_steps
    adapter = Adapter(config, output_dir, episode_id=episode_id)
    adapter.description.metadata.update(arm_roles={n: a.role for n, a in installation.robot.arms.items()},
                                        deployment=audit)
    try:
        info = EpisodeInfo(episode_id=episode_id, task=instruction, integration="piper_x",
            environment=adapter.description,
            metadata={"instruction": instruction, "skill_namespace": "piper_x", **audit})
        service = EnvironmentService(adapter, info, ArtifactStore(output_dir))
        service.initialize()
        return service
    except BaseException:
        adapter.close()
        raise


def serve(service, output_dir, *, host, port):
    from fastapi import FastAPI
    import uvicorn
    from uni_robot_agent.services.environment.routes import build_router
    if host not in {"127.0.0.1", "localhost"}:
        raise ValueError("Physical environment routes must bind loopback; use an authenticated SSH tunnel remotely")

    def stop(reason):
        result = service.backend.finish(reason)
        write_json_atomic(Path(output_dir) / "environment-outcome.json", result)

    @asynccontextmanager
    async def lifespan(app):
        try:
            yield
        finally:
            try:
                stop("server_shutdown")
            finally:
                service.close()

    class Server(uvicorn.Server):
        def handle_exit(self, sig, frame):
            threading.Thread(target=stop, args=("operator_cancelled",), daemon=True).start()
            super().handle_exit(sig, frame)

    app = FastAPI(lifespan=lifespan)
    app.include_router(build_router(service))
    try:
        Server(uvicorn.Config(app, host=host, port=port)).run()
    finally:
        try:
            stop("server_shutdown")
        finally:
            service.close()
