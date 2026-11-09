"""Bounded depth completion of immutable inputs and its reusable, cancellable client child process."""

from __future__ import annotations

import base64
import hashlib
import io
import json
import math
import os
import selectors
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path
from uuid import uuid4

import numpy as np


def checkpoint(cancel, deadline):
    if cancel is not None and cancel.is_set():
        raise InterruptedError('model processing cancelled')
    if time.monotonic() >= deadline:
        raise TimeoutError('model processing deadline exceeded')


def load_config(path):
    path = Path(path).expanduser().resolve()
    config = json.loads(path.read_text())
    if 'url' in config:
        from urllib.parse import urlsplit
        url = urlsplit(config['url'])
        if set(config) - {'url', 'timeout_s'} or url.scheme not in {'http','https'} or not url.hostname:
            raise ValueError('invalid shared depth service configuration')
        budget = config.get('timeout_s',120)
        if type(budget) not in (int,float) or not math.isfinite(budget) or not 0 < budget <= 600:
            raise ValueError('invalid depth service timeout')
        return config
    if set(config) - {'python', 'source', 'weights', 'device', 'tokens', 'timeout_s'}:
        raise ValueError('unknown depth configuration fields')
    for key in ('python', 'source', 'weights'):
        value = Path(config[key]).expanduser()
        # Do not resolve the Python symlink out of its model venv.
        config[key] = os.path.abspath(path.parent / value if not value.is_absolute() else value)
        if not Path(config[key]).exists():
            raise ValueError(f'depth {key} path does not exist')
    if config.get('device', 'auto') not in {'auto', 'cpu', 'cuda', 'mps'}:
        raise ValueError('invalid depth device')
    if type(config.get('tokens', 1200)) is not int or not 1200 <= config.get('tokens', 1200) <= 3600:
        raise ValueError('depth tokens must be in 1200..3600')
    timeout = config.get('timeout_s', 120)
    if type(timeout) not in (int, float) or not math.isfinite(timeout) or not 0 < timeout <= 600:
        raise ValueError('depth timeout_s must be in (0,600]')
    return config


def create_depth_worker(config, directory):
    if 'url' in config:
        return RemoteDepthWorker(config)
    return DepthWorker(config, directory)


def create_depth_backend(config_path, directory):
    """Connect to a resident service (``url``) or start a local worker in the configured model environment."""
    config = load_config(config_path)
    if 'url' in config:
        return create_depth_worker(config, directory)
    import uni_robot_agent

    # The worker runs in its own model environment and imports this checkout.
    root = str(Path(uni_robot_agent.__file__).resolve().parents[1])
    existing = os.environ.get('PYTHONPATH')
    os.environ['PYTHONPATH'] = f'{root}{os.pathsep}{existing}' if existing else root
    entry = Path(__file__).with_name('worker_entry.py')
    return DepthWorker(config, directory, command=[config['python'], str(entry),
        '--source', config['source'], '--weights', config['weights'], '--device', config.get('device', 'auto')])


class DepthWorker:
    """One reusable child, serialized requests, bounded queue + startup + inference.

    Timeout/cancellation terminates this exact child before another request can
    start. The caller must provision the package and model deps in its Python.
    """
    def __init__(self, config, directory, *, command=None):
        self.config = dict(config)
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.command = command or [config['python'], '-m',
            'uni_robot_agent.integrations.arx_one.perception.completion.worker',
            '--source', config['source'], '--weights', config['weights'],
            '--device', config.get('device', 'auto')]
        self.lock = threading.Lock()
        self.process = self.log = None
        self.buffer = b''
        self.closed = threading.Event()

    def _read(self, deadline, cancel):
        with selectors.DefaultSelector() as selector:
            selector.register(self.process.stdout, selectors.EVENT_READ)
            while b'\n' not in self.buffer:
                checkpoint(cancel, deadline)
                if self.closed.is_set(): raise InterruptedError('depth worker closed')
                if selector.select(min(.05, max(0, deadline-time.monotonic()))):
                    chunk = os.read(self.process.stdout.fileno(), 65536)
                    if not chunk: raise RuntimeError('depth worker exited; see depth-worker.log')
                    self.buffer += chunk
                    if len(self.buffer) > 4 * 1024**2: raise ValueError('oversized worker response')
            line, self.buffer = self.buffer.split(b'\n', 1)
            return json.loads(line)

    def _stop(self):
        process, self.process = self.process, None
        if process:
            if process.poll() is None:
                process.terminate()
                try: process.wait(timeout=1)
                except subprocess.TimeoutExpired:
                    process.kill(); process.wait(timeout=1)
            process.stdin.close(); process.stdout.close()
        if self.log: self.log.close(); self.log = None
        self.buffer = b''

    def predict(self, snapshot, *, observe_id, camera, cancel=None, timeout_s=None):
        return self._predict_many([(camera, snapshot)], observe_id=observe_id,
            cancel=cancel, timeout_s=timeout_s, batch=False)[0]

    def predict_many(self, snapshots, *, observe_id, cancel=None, timeout_s=None):
        return self._predict_many(list(snapshots.items()), observe_id=observe_id,
            cancel=cancel, timeout_s=timeout_s, batch=True)

    def _predict_many(self, snapshots, *, observe_id, cancel, timeout_s, batch):
        if not 1 <= len(snapshots) <= 3:
            raise ValueError('depth batch requires one to three views')
        budget = self.config.get('timeout_s', 120) if timeout_s is None else timeout_s
        if type(budget) not in (int, float) or not math.isfinite(budget) or not 0 < budget <= 600:
            raise ValueError('invalid depth timeout')
        deadline = time.monotonic() + budget
        began = time.monotonic()
        while not self.lock.acquire(timeout=.05):
            checkpoint(cancel, deadline)
            if self.closed.is_set(): raise InterruptedError('depth worker closed')
        try:
            checkpoint(cancel, deadline)
            if self.closed.is_set(): raise InterruptedError('depth worker closed')
            queue_s = time.monotonic() - began
            cold = self.process is None or self.process.poll() is not None
            if cold:
                self._stop()
                self.log = (self.directory/'depth-worker.log').open('ab')
                self.process = subprocess.Popen(self.command, stdin=subprocess.PIPE,
                    stdout=subprocess.PIPE, stderr=self.log, bufsize=0)
                if self._read(deadline, cancel).get('ready') is not True:
                    raise RuntimeError('depth worker did not become ready')
            with tempfile.TemporaryDirectory(prefix='depth-', dir=self.directory) as tmp:
                def prepare(entry):
                    camera, snapshot = entry
                    directory = Path(tmp)/uuid4().hex
                    directory.mkdir()
                    capture = directory/'capture.npz'
                    np.savez(capture, rgb_jpeg=np.frombuffer(snapshot.jpeg, dtype=np.uint8),
                        depth_mm=snapshot.depth_mm, intrinsics_json=json.dumps(snapshot.intrinsics),
                        metadata_json=json.dumps({**snapshot.metadata, 'observe_id': observe_id,
                            'role': camera, 'unit': 'mm', 'aligned_to': 'color'}))
                    return dict(request_id=uuid4().hex, observe_id=observe_id, camera=camera,
                        input_sha256=hashlib.sha256(capture.read_bytes()).hexdigest(),
                        capture=str(capture), tokens=self.config.get('tokens',1200))
                from concurrent.futures import ThreadPoolExecutor
                with ThreadPoolExecutor(max_workers=2) as pool:
                    requests = list(pool.map(prepare, snapshots))
                checkpoint(cancel, deadline)
                request = {'batch': requests} if batch else requests[0]
                self.process.stdin.write((json.dumps(request)+'\n').encode()); self.process.stdin.flush()
                reply = self._read(deadline, cancel)
                checkpoint(cancel, deadline)
                if not reply.get('ok'): raise RuntimeError(reply.get('error', 'depth inference failed'))
                replies = reply.get('batch', []) if batch else [reply]
                if len(replies) != len(requests): raise ValueError('depth batch reply count mismatch')
                results = []
                for request, reply, (_, snapshot) in zip(requests, replies, snapshots):
                    if any(reply.get(k) != request[k] for k in ('request_id','observe_id','camera','input_sha256')):
                        raise ValueError('depth response input identity mismatch')
                    with np.load(Path(request['capture']).parent/'completion.npz', allow_pickle=False) as arrays:
                        model = arrays['model_m'].copy()
                    if model.shape != snapshot.depth_mm.shape or model.dtype.kind != 'f':
                        raise ValueError('depth model shape/type mismatch')
                    results.append((model, {**reply.get('report', {}), 'worker_cold_start':cold,
                        'queue_s':queue_s, 'elapsed_s':time.monotonic()-began}))
                checkpoint(cancel, deadline)
                return results
        except BaseException:
            self._stop()
            raise
        finally:
            self.lock.release()

    def close(self):
        self.closed.set()
        # Active predict checks closed every 50 ms and reaps the child itself.
        if self.lock.acquire(timeout=3):
            try: self._stop()
            finally: self.lock.release()


class CompletionService:
    def __init__(self, store, backend):
        self.store, self.backend = store, backend

    def complete(self, observe_id, camera, *, cancel=None, timeout_s=None):
        snapshot = self.store.snapshot(observe_id, camera)
        try:
            model, report = self.backend.predict(snapshot, observe_id=observe_id,
                camera=camera, cancel=cancel, timeout_s=timeout_s)
            if cancel is not None and cancel.is_set(): raise InterruptedError('completion cancelled')
            model = np.asarray(model)
            if model.shape != snapshot.depth_mm.shape or model.dtype.kind not in 'fiu':
                raise ValueError('model depth dimensions/type differ from original RGB-D')
            valid_model = np.isfinite(model) & (model > 0) & (model <= 65.535)
            model = np.where(valid_model, model, 0).astype(np.float64)
            measured = snapshot.depth_mm.astype(np.float64) / 1000.
            valid_measured = snapshot.depth_mm > 0
            filled = np.where(valid_measured, measured, model)
            source = np.where(valid_measured, 1, np.where(valid_model, 2, 0)).astype(np.uint8)
            return self.store.put_derived(observe_id, camera, kind='depth_completion',
                arrays={'measured_m':measured, 'model_m':model, 'filled_m':filled, 'source_mask':source},
                metadata={'unit':'m', 'aligned_to':'color', 'source_labels':{'0':'invalid','1':'measured','2':'model'},
                    'report':report, 't_request':snapshot.metadata.get('t_request'),
                    'source_times':snapshot.metadata.get('source_times'),
                    'geometry_current':False, 'motion_ready':False})
        except Exception as exc:
            self.store.processing_event(observe_id, 'depth_failed', camera=camera, error=str(exc))
            raise

    def close(self): self.backend.close()


class RemoteDepthWorker:
    def __init__(self, config):
        self.config = dict(config)
        self.closed = threading.Event()
        self.lock = threading.Lock()
        self.process = None

    def _check(self, cancel, deadline):
        checkpoint(cancel, deadline)
        if self.closed.is_set():
            raise InterruptedError('depth client closed')

    def _stop(self):
        process, self.process = self.process, None
        if process is not None:
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=1)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=1)
            process.stdin.close()
            process.stdout.close()

    def _reply(self, cancel, deadline):
        chunks = bytearray()
        with selectors.DefaultSelector() as selector:
            selector.register(self.process.stdout, selectors.EVENT_READ)
            while True:
                self._check(cancel, deadline)
                if selector.select(min(.05, max(0, deadline - time.monotonic()))):
                    chunk = os.read(self.process.stdout.fileno(), 4096)
                    if not chunk:
                        raise RuntimeError('depth request helper exited')
                    chunks.extend(chunk)
                    if len(chunks) > 65536:
                        raise ValueError('oversized depth helper response')
                    if b'\n' in chunks:
                        return json.loads(chunks)

    def predict(self, snapshot, *, observe_id, camera, cancel=None, timeout_s=None):
        return self._predict_many([(camera, snapshot)], observe_id=observe_id,
            cancel=cancel, timeout_s=timeout_s, batch=False)[0]

    def predict_many(self, snapshots, *, observe_id, cancel=None, timeout_s=None):
        return self._predict_many(list(snapshots.items()), observe_id=observe_id,
            cancel=cancel, timeout_s=timeout_s, batch=True)

    def _predict_many(self, snapshots, *, observe_id, cancel, timeout_s, batch):
        if not 1 <= len(snapshots) <= 3:
            raise ValueError('depth batch requires one to three views')
        budget = self.config.get('timeout_s', 120) if timeout_s is None else timeout_s
        if type(budget) not in (int, float) or not math.isfinite(budget) or not 0 < budget <= 600:
            raise ValueError('invalid depth timeout')
        started = time.monotonic()
        deadline = started + budget
        while not self.lock.acquire(timeout=.05):
            self._check(cancel, deadline)
        try:
            self._check(cancel, deadline)
            queue_s = time.monotonic() - started
            cold = self.process is None or self.process.poll() is not None
            if cold:
                self._stop()
                self.process = subprocess.Popen(
                    [sys.executable, '-m', __name__], stdin=subprocess.PIPE,
                    stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, bufsize=0)
            def prepare(entry):
                camera, snapshot = entry
                buffer = io.BytesIO()
                np.savez_compressed(buffer, depth_mm=snapshot.depth_mm)
                return {'request_id': uuid4().hex, 'observe_id': observe_id, 'camera': camera,
                        'jpeg': base64.b64encode(snapshot.jpeg).decode(),
                        'depth': base64.b64encode(buffer.getvalue()).decode(),
                        'intrinsics': snapshot.intrinsics, 'metadata': snapshot.metadata,
                        'timeout_s': min(budget, 120)}
            from concurrent.futures import ThreadPoolExecutor
            with ThreadPoolExecutor(max_workers=2) as pool:
                requests = list(pool.map(prepare, snapshots))
            identity = ({'request_id': uuid4().hex, 'observe_id': observe_id, 'camera': '__batch__'}
                        if batch else {k: requests[0][k] for k in ('request_id','observe_id','camera')})
            data = {**identity, 'batch': requests, 'timeout_s': min(budget,120)} if batch else requests[0]
            with tempfile.TemporaryDirectory(prefix='rua-depth-request-') as directory:
                path = Path(directory)
                (path / 'request.json').write_text(json.dumps(data))
                self._check(cancel, deadline)
                control = {'directory': directory, 'url': self.config['url'],
                           'timeout': deadline - time.monotonic(), **identity}
                message = (json.dumps(control) + '\n').encode()
                if len(message) > 4096:
                    raise ValueError('oversized depth helper request')
                self.process.stdin.write(message)
                reply = self._reply(cancel, deadline)
                if any(reply.get(k) != v for k, v in identity.items()):
                    raise ValueError('depth helper identity mismatch')
                if not reply.get('ok'):
                    raise RuntimeError('depth service request failed: ' + reply.get('error', 'unknown'))
                reply = json.loads((path / 'response.json').read_bytes())
                if any(reply.get(k) != v for k, v in identity.items()):
                    raise ValueError('depth service identity mismatch')
                replies = reply.get('batch', []) if batch else [reply]
                if len(replies) != len(requests):
                    raise ValueError('depth batch reply count mismatch')
                results = []
                for item, request, (_, snapshot) in zip(replies, requests, snapshots):
                    if any(item.get(k) != request[k] for k in ('request_id','observe_id','camera')):
                        raise ValueError('depth service identity mismatch')
                    with np.load(io.BytesIO(base64.b64decode(item['model'])), allow_pickle=False) as arrays:
                        model = arrays['model_m'].copy()
                    if model.shape != snapshot.depth_mm.shape or model.dtype.kind != 'f':
                        raise ValueError('depth service returned invalid model array')
                    results.append((model, {**item['report'], 'client_cold_start': cold,
                        'client_queue_s': queue_s, 'client_elapsed_s': time.monotonic() - started}))
                self._check(cancel, deadline)
                return results
        except BaseException:
            # No stale reply may cross into another observation after an error.
            self._stop()
            raise
        finally:
            self.lock.release()

    def close(self):
        self.closed.set()
        # In-flight reads check closed every 50 ms, then reap this child only.
        if self.lock.acquire(timeout=3):
            try:
                self._stop()
            finally:
                self.lock.release()


def main():
    import http.client
    import select
    from urllib.parse import urlsplit
    connection = origin = None
    last_response_at = None
    try:
        for line in sys.stdin.buffer:
            request = json.loads(line)
            identity = {k: request[k] for k in ('request_id', 'observe_id', 'camera')}
            try:
                url = urlsplit(request['url'])
                if url.scheme not in {'http', 'https'} or not url.hostname:
                    raise ValueError('invalid depth service URL')
                current_origin = (url.scheme, url.hostname, url.port)
                # Servers can close keep-alive sockets while the Agent thinks.
                # Discard idle/closed sockets BEFORE submitting a new request;
                # never replay a possibly submitted prediction after an error.
                if connection is not None and (
                    connection.sock is None
                    or last_response_at is None
                    or time.monotonic() - last_response_at >= 1.0
                    or select.select([connection.sock], [], [], 0)[0]
                ):
                    connection.close()
                    connection = None
                if connection is None or origin != current_origin:
                    if connection is not None:
                        connection.close()
                    cls = http.client.HTTPSConnection if url.scheme == 'https' else http.client.HTTPConnection
                    connection = cls(url.hostname, url.port, timeout=request['timeout'])
                    origin = current_origin
                connection.timeout = request['timeout']
                if connection.sock is not None:
                    connection.sock.settimeout(request['timeout'])
                path = Path(request['directory'])
                target = url.path or '/'
                if url.query:
                    target += '?' + url.query
                connection.request('POST', target, (path / 'request.json').read_bytes(),
                                   {'Content-Type': 'application/json'})
                response = connection.getresponse()
                payload = response.read()
                last_response_at = time.monotonic()
                if response.status != 200:
                    raise RuntimeError(f'HTTP {response.status}')
                (path / 'response.json').write_bytes(payload)
                reply = {**identity, 'ok': True}
            except Exception as exc:
                if connection is not None:
                    connection.close()
                connection = None
                reply = {**identity, 'ok': False, 'error': f'{type(exc).__name__}: {exc}'}
            print(json.dumps(reply), flush=True)
    finally:
        if connection is not None:
            connection.close()


if __name__ == '__main__':
    main()
