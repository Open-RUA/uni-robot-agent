"""JSON-line LingBot worker with bounded CPU overlap; runs inside a separately provisioned model environment."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path


def run_pipeline(items, prepare, infer, finish):
    """Process at most three views, preserving input order and joining on error.

    Only prepare/finish run in threads. infer always runs serially on the caller.
    No retries or partial-success publication. The caller owns cancellation of
    its worker process and must keep all input/output directories alive until
    this function returns (including failure cleanup).
    """
    items = tuple(items)
    if not 1 <= len(items) <= 3:
        raise ValueError('depth pipeline requires one to three views')
    with ThreadPoolExecutor(max_workers=2, thread_name_prefix='depth-cpu') as pool:
        prepared = [pool.submit(prepare, item) for item in items]
        finished = []
        try:
            for future in prepared:
                for completed in finished:
                    if completed.done():
                        completed.result()
                frame = future.result()
                result = infer(frame)
                finished.append(pool.submit(finish, frame, result))
            return [future.result() for future in finished]
        except BaseException:
            for future in (*prepared, *finished):
                future.cancel()
            raise


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source', type=Path, required=True)
    p.add_argument('--weights', type=Path, required=True)
    p.add_argument('--device', choices=['auto', 'cpu', 'cuda', 'mps'], default='auto')
    args = p.parse_args()
    # Replies own the original stdout. File descriptor 1 now points at stderr, so model
    # libraries writing there (including native torch and CUDA code) cannot corrupt them.
    replies_out = os.fdopen(os.dup(sys.stdout.fileno()), "w", buffering=1)
    sys.stdout.flush()
    os.dup2(sys.stderr.fileno(), sys.stdout.fileno())
    import torch
    from .lingbot import load_model, prepare_frame, infer_frame, finish_frame
    device = args.device
    if device == 'auto':
        device = 'cuda' if torch.cuda.is_available() else 'mps' if torch.backends.mps.is_available() else 'cpu'
    model, info = load_model(args.source, args.weights, device)
    print(json.dumps({'ready': True, 'model': info}), file=replies_out, flush=True)
    for line in sys.stdin:
        try:
            request = json.loads(line)
            # Existing single-frame protocol remains supported. A bounded batch
            # is one transaction; replies retain each input's full identity.
            batch = request.get('batch')
            items = batch if batch is not None else [request]
            if not isinstance(items, list) or not 1 <= len(items) <= 3:
                raise ValueError('depth batch requires one to three views')
            identities = [tuple(item[k] for k in ('request_id', 'observe_id', 'camera')) for item in items]
            if len(set(identities)) != len(items):
                raise ValueError('duplicate depth input identity')
            if batch is not None and len({item['observe_id'] for item in items}) != 1:
                raise ValueError('depth batch must belong to one observation')
            paths = [Path(item['capture']).resolve() for item in items]
            if len({path.parent for path in paths}) != len(paths):
                raise ValueError('depth outputs require separate directories')

            def prepare(item):
                capture = Path(item['capture'])
                if hashlib.sha256(capture.read_bytes()).hexdigest() != item['input_sha256']:
                    raise ValueError('capture checksum mismatch')
                tokens = item['tokens']
                if type(tokens) is not int or not 1200 <= tokens <= 3600:
                    raise ValueError('tokens must be in 1200..3600')
                frame = prepare_frame(capture, tokens)
                if (frame['meta'].get('observe_id') != item['observe_id'] or
                        frame['meta'].get('role') != item['camera']):
                    raise ValueError('capture metadata identity mismatch')
                return frame

            reports = run_pipeline(items, prepare,
                lambda frame: infer_frame(model, frame, info),
                lambda frame, result: finish_frame(frame, result, info))
            replies = [dict(ok=True, **{key: item[key] for key in
                ('request_id', 'input_sha256', 'observe_id', 'camera')}, report=report)
                for item, report in zip(items, reports)]
            print(json.dumps({'ok': True, 'batch': replies} if batch is not None else replies[0]),
                  file=replies_out, flush=True)
        except Exception as exc:
            print(json.dumps({'ok': False, 'error': f'{type(exc).__name__}: {exc}'}), file=replies_out, flush=True)


if __name__ == '__main__':
    main()
