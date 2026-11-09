"""Resident depth-completion service shared by runs: prediction over posted RGB-D inputs and its HTTP host."""

import argparse
import base64
import io
import logging
import threading
import time
import zipfile
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import numpy as np
import uvicorn
from PIL import Image
from fastapi import FastAPI
from fastapi.responses import JSONResponse

from ..observation_store import ObservationStore
from ..snapshot import RGBDSnapshot
from .client import CompletionService, create_depth_backend


class DepthPrediction:
    @staticmethod
    def _decode_depth(body):
        from ..snapshot import RGBDSnapshot
        import time
        import math
        required={'request_id','observe_id','camera','jpeg','depth','intrinsics','metadata'}
        if not isinstance(body,dict) or not required <= body.keys() or set(body)-required-{'timeout_s'}:
            raise ValueError('invalid shared depth request fields')
        if any(not isinstance(body[k],str) or not 0 < len(body[k]) <= 512
               for k in ('request_id','observe_id','camera')):
            raise ValueError('invalid shared depth identity')
        if not isinstance(body['intrinsics'],dict) or not isinstance(body['metadata'],dict):
            raise ValueError('invalid shared depth metadata')
        budget=body.get('timeout_s',120)
        if type(budget) not in (int,float) or not math.isfinite(budget) or not 0 < budget <= 120:
            raise ValueError('shared depth timeout must be in (0,120]')
        if any(not isinstance(body[k],str) or len(body[k]) > 24*1024*1024 for k in ('jpeg','depth')):
            raise ValueError('oversized shared depth input')
        raw=base64.b64decode(body['depth'],validate=True)
        # Check the uncompressed size before numpy inflates the archive.
        with zipfile.ZipFile(io.BytesIO(raw)) as archive:
            if (archive.namelist()!=['depth_mm.npy']
                    or archive.getinfo('depth_mm.npy').file_size>4096*4096*2+4096):
                raise ValueError('invalid depth archive')
        with np.load(io.BytesIO(raw),allow_pickle=False) as data:
            depth=data['depth_mm'].copy()
        jpeg=base64.b64decode(body['jpeg'],validate=True)
        if depth.ndim!=2 or depth.size>4096*4096 or depth.dtype!=np.uint16:raise ValueError('invalid depth array')
        with Image.open(io.BytesIO(jpeg)) as image:
            if image.size!=(depth.shape[1],depth.shape[0]):raise ValueError('RGB-D dimensions differ')
        return RGBDSnapshot(jpeg,depth,body['intrinsics'],body['metadata'],time.monotonic())

    def predict(self, body):
        import time
        import math
        started = time.monotonic()
        batch = isinstance(body, dict) and 'batch' in body
        if batch:
            if set(body) != {'request_id','observe_id','camera','batch','timeout_s'}:
                raise ValueError('invalid depth batch fields')
            if any(not isinstance(body[k],str) or not 0 < len(body[k]) <= 512
                   for k in ('request_id','observe_id','camera')):
                raise ValueError('invalid depth batch identity')
            items = body['batch']
            if not isinstance(items,list) or not 1 <= len(items) <= 3:
                raise ValueError('depth batch requires one to three views')
            if any(not isinstance(item,dict) or item.get('observe_id') != body['observe_id'] for item in items):
                raise ValueError('depth batch observation mismatch')
            cameras = [item.get('camera') for item in items]
            ids = [item.get('request_id') for item in items]
            if any(not isinstance(v,str) for v in cameras+ids) or len(set(cameras)) != len(items) or len(set(ids)) != len(items):
                raise ValueError('duplicate or invalid depth batch identity')
        else:
            items = [body]
        budget = body.get('timeout_s',120) if isinstance(body,dict) else None
        if type(budget) not in (int,float) or not math.isfinite(budget) or not 0 < budget <= 120:
            raise ValueError('shared depth timeout must be in (0,120]')
        if self.closed:raise RuntimeError('workbench closed')
        if not self.depth_admission.acquire(blocking=False):raise RuntimeError('depth service queue full')
        try:
            with ThreadPoolExecutor(max_workers=2) as pool:
                snapshots = list(pool.map(self._decode_depth, items))
                remaining = budget - (time.monotonic()-started)
                if remaining <= 0:raise TimeoutError('depth preparation deadline exceeded')
                if batch:
                    results = self.depth.backend.predict_many(dict(zip(cameras,snapshots)),
                        observe_id=body['observe_id'],timeout_s=remaining)
                else:
                    results = [self.depth.backend.predict(snapshots[0],observe_id=body['observe_id'],
                        camera=body['camera'],timeout_s=remaining)]
                if len(results) != len(items):raise ValueError('depth batch result count mismatch')
                def encode(entry):
                    item, (model, report) = entry
                    buffer=io.BytesIO();np.savez_compressed(buffer,model_m=model)
                    return {k:item[k] for k in ('request_id','observe_id','camera')} | {
                        'model':base64.b64encode(buffer.getvalue()).decode(),'report':report}
                replies=list(pool.map(encode,zip(items,results)))
            if time.monotonic()-started >= budget:raise TimeoutError('depth result deadline exceeded')
            return ({k:body[k] for k in ('request_id','observe_id','camera')} | {'batch':replies}) if batch else replies[0]
        finally:self.depth_admission.release()


logger = logging.getLogger(__name__)


class DepthService(DepthPrediction):
    """Shared depth prediction for runs, without observation capture or segmentation."""

    def __init__(self, config_path, root):
        self.store = ObservationStore(root)
        self.depth = CompletionService(self.store, create_depth_backend(config_path, self.store.root / "worker"))
        self.closed = False
        self.depth_admission = threading.BoundedSemaphore(2)
        self.warm = {"state": "pending"}

    def warm_up(self):
        """Load the model before the first run needs it; a failure is reported, not fatal."""
        started = time.monotonic()
        buffer = io.BytesIO()
        Image.new("RGB", (640, 480), (128, 128, 128)).save(buffer, format="JPEG")
        depth = np.full((480, 640), 500, dtype=np.uint16)
        intrinsics = {"color": {"width": 640, "height": 480, "fx": 400., "fy": 400., "cx": 320., "cy": 240.,
                                "model": "none"}}
        snapshot = RGBDSnapshot(buffer.getvalue(), depth, intrinsics, {"unit": "mm", "aligned_to": "color"},
                                time.monotonic())
        try:
            self.depth.backend.predict(snapshot, observe_id="warm-up", camera="warm-up", timeout_s=300)
            self.warm = {"state": "ready", "seconds": round(time.monotonic() - started, 1)}
        except Exception as error:
            self.warm = {"state": "failed", "error": f"{type(error).__name__}: {error}"[:300]}
            logger.warning("Depth warm-up failed: %s", self.warm["error"])

    def close(self):
        self.closed = True
        self.depth.close()


def create_app(service):
    app = FastAPI(docs_url=None, redoc_url=None)

    @app.exception_handler(ValueError)
    @app.exception_handler(RuntimeError)
    @app.exception_handler(InterruptedError)
    async def rejected(request, error):
        return JSONResponse({"error": str(error)}, status_code=409)

    @app.exception_handler(TimeoutError)
    async def timed_out(request, error):
        return JSONResponse({"error": str(error)}, status_code=504)

    @app.get("/api/status")
    def status():
        return {"depth_service": {"shared": True, "endpoint": "/api/depth/predict"}, "model": service.warm}

    @app.post("/api/depth/predict")
    def predict(body: dict):
        return service.predict(body)

    return app


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--depth-config", type=Path, required=True, help="Local model configuration")
    parser.add_argument("--state-dir", type=Path, required=True)
    parser.add_argument("--port", type=int, default=18875)
    args = parser.parse_args()
    logging.basicConfig(level=logging.INFO)
    service = DepthService(args.depth_config, args.state_dir)
    threading.Thread(target=service.warm_up, daemon=True).start()
    try:
        uvicorn.run(create_app(service), host="127.0.0.1", port=args.port, log_level="warning")
    finally:
        service.close()


if __name__ == "__main__":
    main()
