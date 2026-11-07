"""Local read-only run collection, JSON state, SSE and image routes."""
from __future__ import annotations

import asyncio
from pathlib import Path
import hashlib
import json
import io
from functools import lru_cache
import threading
from urllib.parse import quote

from fastapi import FastAPI, HTTPException, Request
from fastapi.responses import FileResponse, StreamingResponse, Response
from starlette.concurrency import run_in_threadpool

from uni_robot_agent.artifacts.discovery import discover
from .reader import RunReader

STATIC = Path(__file__).parent / 'static'
IMAGE_SUFFIXES = {'.png', '.jpg', '.jpeg', '.webp', '.gif', '.bmp'}


@lru_cache(maxsize=128)
def preview_image(path, modified_ns, width):
    from PIL import Image
    with Image.open(path) as original:
        image = original.convert('RGB')
        image.thumbnail((width, width))
        stream = io.BytesIO()
        image.save(stream, format='JPEG', quality=82)
        return stream.getvalue()


class Collection:
    def __init__(self, roots, archive_scopes=()):
        self.readers = {hashlib.sha256(str(Path(p).resolve()).encode()).hexdigest()[:16]: RunReader(p, archive_scopes) for p in roots}
        self.closing = threading.Event()

    def reader(self, run_id):
        try:
            return self.readers[run_id]
        except KeyError:
            raise HTTPException(404, 'Unknown run') from None

    def state(self, run_id):
        reader = self.reader(run_id)
        state = dict(reader.refresh())
        state['id'] = run_id
        state['directory'] = str(reader.root)
        state['frames'] = [dict(f, url=f'/api/runs/{run_id}/media/{quote(f["path"], safe="/")}') for f in state['frames']]
        return state

    def overview(self):
        entries = []
        for run_id in self.readers:
            state = self.state(run_id)
            entries.append({key: state[key] for key in ('id', 'name', 'task', 'integration', 'model', 'status',
                'started_at', 'finished_at', 'elapsed_s', 'steps', 'max_steps', 'score', 'task_success', 'directory')}
                | {'frame': state['frames'][-1] if state['frames'] else None})
        return entries


def create_monitor_app(collection):
    app = FastAPI(docs_url=None, redoc_url=None, openapi_url=None)

    @app.get('/api/runs')
    def runs():
        return collection.overview()

    @app.get('/api/runs/{run_id}')
    def state(run_id: str):
        return collection.state(run_id)

    @app.get('/api/runs/{run_id}/events')
    async def events(run_id: str, request: Request):
        collection.reader(run_id)
        async def stream():
            # Full idempotent state on reconnect; no event loss or duplicate call insertion.
            revision = None
            while not collection.closing.is_set():
                if await request.is_disconnected():
                    break
                state = await run_in_threadpool(collection.state, run_id)
                if state['revision'] != revision:
                    revision = state['revision']
                    yield f'id: {revision}\nevent: state\ndata: {json.dumps(state, ensure_ascii=False, allow_nan=False)}\n\n'
                else:
                    yield ': heartbeat\n\n'
                await asyncio.sleep(0.5)
            final = await run_in_threadpool(collection.state, run_id)
            yield f'event: state\ndata: {json.dumps(final, ensure_ascii=False)}\n\n'
        return StreamingResponse(stream(), media_type='text/event-stream',
            headers={'Cache-Control': 'no-cache', 'X-Accel-Buffering': 'no'})

    @app.get('/api/runs/{run_id}/media/{relative:path}')
    def media(run_id: str, relative: str, width: int | None = None):
        reader = collection.reader(run_id)
        allowed = {f['path'] for f in reader.refresh()['frames']}
        if relative not in allowed:
            raise HTTPException(404, 'Image unavailable')
        path = reader.archive.path(relative)
        if path is None:
            path = (reader.root / relative).resolve()
            if not path.is_relative_to(reader.root):
                raise HTTPException(404, 'Image unavailable')
        if path.suffix.lower() not in IMAGE_SUFFIXES or not path.is_file():
            raise HTTPException(404, 'Image unavailable')
        if width is not None:
            if width not in (320, 960):
                raise HTTPException(400, 'Unsupported preview size')
            return Response(preview_image(str(path), path.stat().st_mtime_ns, width), media_type='image/jpeg',
                            headers={'Cache-Control': 'private, max-age=3600', 'X-Content-Type-Options': 'nosniff'})
        return FileResponse(path, headers={'Cache-Control': 'private, max-age=3600', 'X-Content-Type-Options': 'nosniff'})

    @app.get('/')
    def index():
        return FileResponse(STATIC / 'index.html', headers={'Cache-Control': 'no-cache'})

    @app.get('/static/{name}')
    def static(name: str):
        path = STATIC / name
        if path.parent != STATIC or path.suffix not in {'.js', '.css'} or not path.is_file():
            raise HTTPException(404)
        return FileResponse(path, headers={'Cache-Control': 'no-cache'})

    return app
