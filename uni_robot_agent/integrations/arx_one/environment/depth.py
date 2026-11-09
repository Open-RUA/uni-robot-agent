"""Optional model fill on the exact captured RGB-D; measured pixels are never replaced."""
from dataclasses import replace
import io
import time

import numpy as np
from PIL import Image

from ..perception.snapshot import RGBDSnapshot


def _png(rgb):
    stream = io.BytesIO()
    Image.fromarray(rgb).save(stream, format="PNG")
    return stream.getvalue()


def _intrinsics(frame):
    k = frame.intrinsic_K
    if k is None:
        return {}
    # Frames are already rectified, so the model receives a distortion-free pinhole.
    return {"color": {"fx": float(k[0, 0]), "fy": float(k[1, 1]), "cx": float(k[0, 2]), "cy": float(k[1, 2]),
                      "width": frame.rgb.shape[1], "height": frame.rgb.shape[0], "model": "none", "d": [0.] * 5}}


def _snapshot(name, frame, valid, source, observe_id):
    millimeters = np.where(valid, np.clip(np.rint(frame.depth_m * 1000), 0, 65535), 0).astype(np.uint16)
    return RGBDSnapshot(_png(frame.rgb), millimeters, _intrinsics(frame),
        {"observe_id": observe_id, "camera": name, "unit": "mm", "aligned_to": "color",
         "sequence": source.get("sequence"), "rectified": source.get("rectified", False),
         "camera_provenance": source}, time.monotonic())


def complete_frames(frames, provenance, backend, *, observe_id):
    """Return frames with holes filled, plus a report for the snapshot state."""
    prepared = {}
    for name, frame in frames.items():
        if frame.depth_m is None:
            continue
        valid = np.isfinite(frame.depth_m) & (frame.depth_m > 0)
        prepared[name] = (valid, _snapshot(name, frame, valid, provenance.get(name, {}), observe_id))
    if not prepared:
        return dict(frames), {"status": "not_applicable"}
    results = backend.predict_many({name: item[1] for name, item in prepared.items()}, observe_id=observe_id)
    if len(results) != len(prepared):
        raise ValueError("Depth batch result count mismatch")
    completed, reports = dict(frames), {}
    for (name, (valid, _)), (model, report) in zip(prepared.items(), results):
        measured = frames[name].depth_m
        model = np.asarray(model)
        if model.shape != measured.shape:
            raise ValueError("Depth model shape mismatch")
        model = np.where(np.isfinite(model) & (model > 0) & (model <= 65.535), model, 0)
        completed[name] = replace(frames[name], depth_m=np.where(valid, measured, model).astype(measured.dtype))
        reports[name] = {"depth_source": "filled", "measured_preserved": True, "report": report,
                         "filled_pixels": int(np.count_nonzero(~valid & (model > 0)))}
    return completed, {"status": "completed", "observe_id": observe_id, "cameras": reports}
