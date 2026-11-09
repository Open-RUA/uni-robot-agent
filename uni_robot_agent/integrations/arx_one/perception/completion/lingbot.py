"""Pinned LingBot inference backend; imported only inside the model worker.

No camera or controller access.
"""

from __future__ import annotations

import hashlib
import io
import json
import os
import resource
import sys
import threading
import time
import types
import warnings

import cv2
import numpy as np
import torch
from PIL import Image, ImageDraw


os.environ["XFORMERS_DISABLED"] = "1"

REVISION = "f3a237e434ae987bc38281476d6cfb5df3e4d739"
WEIGHT_SHA = "b60cf27ddbd0e51e9b59b03475c0d39d02d2e48ecf8dbb5866f04d46802b3c23"


def rectify(rgb, depth, k):
    """Use the same RealSense distortion convention as RUA's measured rays."""
    from ..geometry import pixel_rays

    h, w = depth.shape
    fx, fy, cx, cy = (float(k[key]) for key in ("fx", "fy", "cx", "cy"))
    rows, cols = np.indices((h, w), dtype=np.float32)
    rays = pixel_rays(np.column_stack((rows.ravel(), cols.ravel())), k)
    x, y = (cols - cx) / fx, (rows - cy) / fy
    model = k.get("model", 0)
    if model not in (0, "none", "distortion.none"):
        k1, k2, p1, p2, k3 = k.get("d", k.get("coeffs", [0] * 5))
        r2 = x * x + y * y
        f = 1 + r2 * (k1 + r2 * (k2 + r2 * k3))
        qx, qy = (x * f, y * f) if model in (2, "inverse_brown_conrady", "distortion.inverse_brown_conrady") else (x, y)
        xd = x * f + 2 * p1 * qx * qy + p2 * (r2 + 2 * qx * qx)
        yd = y * f + 2 * p2 * qx * qy + p1 * (r2 + 2 * qy * qy)
    else:
        xd, yd = x, y
    maps = ((xd * fx + cx).astype(np.float32), (yd * fy + cy).astype(np.float32))
    rgb_rect = cv2.remap(rgb, *maps, cv2.INTER_LINEAR)
    depth_rect = cv2.remap(depth, *maps, cv2.INTER_NEAREST)
    back = ((rays[:, 0] * fx + cx).reshape(h, w).astype(np.float32),
            (rays[:, 1] * fy + cy).reshape(h, w).astype(np.float32))
    return rgb_rect, depth_rect, back


def palette(depth, far=1.0):
    valid = np.isfinite(depth) & (depth > 0)
    norm = np.clip((np.where(valid, depth, 0) - .07) / (far - .07), 0, 1)
    colors = cv2.cvtColor(cv2.applyColorMap((norm * 255).astype(np.uint8), cv2.COLORMAP_TURBO), cv2.COLOR_BGR2RGB)
    colors[~valid] = 0
    colors[valid & (depth > far)] = [120, 120, 120]
    return colors


def panel(images, path):
    w, h = images[0][1].shape[1], images[0][1].shape[0]
    canvas = Image.new("RGB", (w * 2, (h + 42) * 2), (20, 23, 29))
    draw = ImageDraw.Draw(canvas)
    for i, (label, image) in enumerate(images):
        x, y = (i % 2) * w, (i // 2) * (h + 42)
        draw.text((x + 10, y + 6), label, fill="white")
        draw.text((x + 10, y + 22), "Depth scale: 0.07-1.00 m; black=invalid, gray=beyond scale", fill=(170, 180, 195))
        canvas.paste(Image.fromarray(image), (x, y + 42))
    canvas.save(path)


def load_model(source, weights, device="mps"):
    if (source / "revision.txt").read_text().strip() != REVISION:
        raise ValueError("unexpected upstream source revision")
    with weights.open("rb") as f:
        digest = hashlib.sha256()
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            digest.update(chunk)
        sha = digest.hexdigest()
    if sha != WEIGHT_SHA:
        raise ValueError("weights do not match the pinned v0.5 release")
    sys.path.insert(0, str(source))
    from mdm.model.dinov2_rgbd.layers.block import Block, NestedTensorBlock
    from mdm.model.v2 import MDMModel

    started = time.monotonic()
    checkpoint = torch.load(weights, map_location="cpu", weights_only=True, mmap=True)
    model = MDMModel(**checkpoint["model_config"])
    model.load_state_dict(checkpoint["model"], strict=True)
    config = checkpoint["model_config"]
    del checkpoint

    def forward(self, value):
        if self.training:
            raise RuntimeError("nested attention adapter is inference-only")
        return [Block.forward(self, item) for item in value] if isinstance(value, list) else Block.forward(self, value)

    patched = 0
    for module in model.modules():
        if isinstance(module, NestedTensorBlock):
            module.forward = types.MethodType(forward, module)
            patched += 1
    model = model.eval().to(device)
    return model, {"upstream_revision": REVISION, "weights_sha256": sha,
                   "torch": torch.__version__, "device": device, "precision": "float32",
                   "parameters": sum(p.numel() for p in model.parameters()),
                   "adapted_nested_blocks": patched, "depth_mask_preserved": True,
                   "load_s": time.monotonic() - started, "model_config": config}


def run_frame(model, capture, tokens, model_info, filter_config=None, diagnostics=False):
    """Compatibility path; identical stages executed sequentially."""
    prepared = prepare_frame(capture, tokens, filter_config)
    result = infer_frame(model, prepared, model_info)
    return finish_frame(prepared, result, model_info, diagnostics)


def prepare_frame(capture, tokens, filter_config=None):
    """CPU only: decode and rectify one immutable input."""
    timing = {"t_processing_started": str(time.time_ns()), "processing_monotonic_ns": time.monotonic_ns()}
    processing_started = time.monotonic()
    out = capture.parent
    with np.load(capture, allow_pickle=False) as data:
        rgb = np.array(Image.open(io.BytesIO(data["rgb_jpeg"].tobytes())).convert("RGB"))
        raw_mm = data["depth_mm"]
        raw = raw_mm.astype(np.float32) / 1000
        intrinsics = json.loads(str(data["intrinsics_json"]))
        meta = json.loads(str(data["metadata_json"]))
        # Explicit, provenance-bearing offline ablations; legacy filtered_mm is
        # never selected implicitly. The sensor array remains separately saved.
        prepared = data["input_depth_mm"].astype(np.float32) / 1000 if "input_depth_mm" in data else None
        preprocessing = json.loads(str(data["preprocessing_json"])) if prepared is not None else None
    if meta.get("unit") != "mm" or meta.get("aligned_to") != "color":
        raise ValueError("capture must contain metric depth registered to RGB")
    if rgb.shape[:2] != raw.shape:
        raise ValueError("RGB and depth must use the same pixel grid")
    valid = np.isfinite(raw) & (raw > 0)
    input_depth = np.where(valid, raw, 0)
    if prepared is not None:
        if filter_config is not None or prepared.shape != raw.shape or not isinstance(preprocessing, dict) or not preprocessing.get("method"):
            raise ValueError("precomputed input requires matching shape, provenance, and no further filter")
        input_depth = np.where(np.isfinite(prepared) & (prepared > 0), prepared, 0)
    elif filter_config is not None:
        from ..depth_filter import filter_depth

        filtered, preprocessing = filter_depth(raw_mm, rgb, filter_config)
        input_depth = filtered.astype(np.float32) / 1000
    compute = {"requested_spatial_tokens": tokens, "source_wh": [raw.shape[1], raw.shape[0]],
               "output_wh": [raw.shape[1], raw.shape[0]], "batch_size": 1}

    if meta.get('rectified') is True:
        # Native main supplies rectified RGB-D even when world extrinsics are
        # unavailable. Model filling needs pixel alignment, not world geometry.
        rows,cols=np.indices(input_depth.shape,dtype=np.float32)
        rgb_rect,depth_rect,back=rgb,input_depth,(cols,rows)
    else:
        rgb_rect, depth_rect, back = rectify(rgb, input_depth, intrinsics["color"])
    return {'timing': timing, 'processing_started': processing_started, 'out': out, 'rgb': rgb, 'raw': raw, 'intrinsics': intrinsics, 'meta': meta, 'preprocessing': preprocessing, 'valid': valid, 'input_depth': input_depth, 'compute': compute, 'rgb_rect': rgb_rect, 'depth_rect': depth_rect, 'back': back, 'tokens': tokens}


def infer_frame(model, prepared, model_info):
    """Single model owner only: transfer, inference, synchronization, CPU copy."""
    timing, compute, rgb_rect, depth_rect, tokens = (prepared[key] for key in
        ('timing', 'compute', 'rgb_rect', 'depth_rect', 'tokens'))
    def encoder_input(module, args):
        rows, cols = int(args[2]), int(args[3])
        compute.update(patch_grid_wh=[cols, rows], encoder_wh=[cols * 14, rows * 14],
                       rgb_tokens=rows * cols, patch_size=14)

    def block_input(module, args):
        values = args[0] if isinstance(args[0], list) else [args[0]]
        lengths = [int(value.shape[-2]) for value in values]
        special = 1 + model.encoder.backbone.num_register_tokens
        compute.update(transformer_sequence_lengths=lengths,
                       depth_tokens=[length - compute["rgb_tokens"] - special for length in lengths],
                       special_tokens=special, embedding_dim=int(values[0].shape[-1]),
                       transformer_blocks=len(model.encoder.backbone.blocks))

    device = model_info["device"]
    image = torch.from_numpy(rgb_rect).permute(2, 0, 1).float().unsqueeze(0).to(device) / 255
    depth = torch.from_numpy(depth_rect).unsqueeze(0).to(device)
    peak = {}
    stopped = threading.Event()

    def sample_memory():
        while not stopped.wait(.05):
            if device == "mps":
                peak["mps_allocated_bytes"] = max(peak.get("mps_allocated_bytes", 0), torch.mps.current_allocated_memory())
                peak["mps_driver_bytes"] = max(peak.get("mps_driver_bytes", 0), torch.mps.driver_allocated_memory())
            elif device == "cuda":
                peak["cuda_allocated_bytes"] = torch.cuda.max_memory_allocated()
            else:
                peak["process_max_rss_bytes"] = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss * (1024 if sys.platform == "linux" else 1)

    def synchronize():
        if device == "mps":
            torch.mps.synchronize()
        elif device == "cuda":
            torch.cuda.synchronize()

    sampler = threading.Thread(target=sample_memory, daemon=True)
    sampler.start()
    synchronize()
    hooks = [model.encoder.register_forward_pre_hook(encoder_input),
             model.encoder.backbone.blocks[0].register_forward_pre_hook(block_input)]
    started = time.monotonic()
    timing["t_inference_started"] = str(time.time_ns())
    try:
        with warnings.catch_warnings(), torch.inference_mode():
            warnings.filterwarnings("ignore", message="In MPS autocast.*")
            prediction = model.infer(image, depth_in=depth, num_tokens=tokens, use_fp16=False)
            synchronize()
        inference_s = time.monotonic() - started
        timing["t_inference_finished"] = str(time.time_ns())
        predicted_rect = prediction["depth"][0].float().cpu().numpy()
    finally:
        for hook in hooks:
            hook.remove()
        stopped.set()
        sampler.join()
    return {"predicted_rect": predicted_rect, "inference_s": inference_s, "peak": peak}


def finish_frame(prepared, result, model_info, diagnostics=False):
    """CPU only: restore pixel grid, diagnostics and artifact compression."""
    timing, processing_started, out, rgb, raw, intrinsics, meta, preprocessing, valid, input_depth, compute, rgb_rect, depth_rect, back, tokens = (prepared['timing'], prepared['processing_started'], prepared['out'], prepared['rgb'], prepared['raw'], prepared['intrinsics'], prepared['meta'], prepared['preprocessing'], prepared['valid'], prepared['input_depth'], prepared['compute'], prepared['rgb_rect'], prepared['depth_rect'], prepared['back'], prepared['tokens'])
    predicted_rect, inference_s, peak = result["predicted_rect"], result["inference_s"], result["peak"]
    refined = cv2.remap(predicted_rect, *back, cv2.INTER_NEAREST)
    model_valid = np.isfinite(refined) & (refined > 0)
    refined = np.where(model_valid, refined, 0).astype(np.float32)
    overlap = valid & model_valid
    differences = np.abs(refined[overlap] - raw[overlap]) * 1000
    timing["t_depth_completed"] = str(time.time_ns())
    report = {"observe_id": meta.get("observe_id"), "t_request": meta.get("t_request"), "timing": timing,
              "camera": meta["role"], "sequence": meta.get("sequence"), "capture_metadata": meta,
              "source_clock_semantics": "capture time retained; completion time is separate",
              "completed_at_unix_s": time.time(), "tokens": tokens, "compute": compute, "inference_s": inference_s,
              "processing_s_before_artifact_write": time.monotonic() - processing_started,
              "raw_valid_fraction": float((raw > 0).mean()),
              "model_valid_fraction": float(model_valid.mean()),
              "model_filled_pixels": int((~valid & model_valid).sum()),
              "input_depth": "filtered_sensor" if preprocessing else "raw_sensor", "output_depth": "model_estimate",
              "input_valid_fraction": float((input_depth > 0).mean()), "preprocessing": preprocessing,
              "filtering": preprocessing is not None, "sensor_fusion": False,
              "model_vs_sensor_abs_p50_p90_mm": np.percentile(differences, [50, 90]).tolist() if differences.size else None,
              "memory_peak_sampled": peak, "process_peak_rss_bytes": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
              "validation": "coverage and consistency only; no geometric ground truth or grasp validation",
              "model": model_info}
    np.savez_compressed(out / "completion.npz", raw_m=raw, input_m=input_depth, model_m=refined,
                        valid=model_valid, metadata_json=json.dumps(report), intrinsics_json=json.dumps(intrinsics))
    if diagnostics:
        overlay = rgb.copy()
        overlay[model_valid] = (rgb[model_valid] * .55 + palette(refined)[model_valid] * .45).astype(np.uint8)
        panel([("RGB (captured source frame)", rgb), ("Raw D405 depth", palette(raw)),
               (f"LingBot model output ({report['input_depth']} input)", palette(refined)),
               ("Model depth on source RGB", overlay)], out / "comparison.png")
        for name, array in (("rgb", rgb), ("model", palette(refined)), ("overlay", overlay)):
            Image.fromarray(array).save(out / f"{name}.png")
        (out / "report.json").write_text(json.dumps(report, indent=2))
    return report
