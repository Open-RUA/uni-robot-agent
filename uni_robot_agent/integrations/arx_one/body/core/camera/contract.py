"""What a camera driver reports so the episode environment can observe through it; checked against ``describe``.

A Body device of kind ``camera`` is usable by ``environment/`` when its ``describe`` entry provides:

Metadata
  ``identity``  A mapping the driver chooses, such as the serial number and model. The environment
                pins it exactly and refuses a Body whose identity differs; facts only, never prose.

Observation stream ``rgbd`` (``read_observation_at``) with the signals
  ``rgb``         ``{"format": "jpeg", "data_b64", "width", "height", "sequence"}``.
  ``depth``       ``{"format": "u16le_b64" | "png16_b64", "unit": "mm", "aligned_to": "color", "data_b64",
                  "width", "height", "sequence"}``: depth registered to the colour grid, 0 where invalid.
                  Optional ``depth_png``, the same depth always as ``png16_b64``; read instead when declared.
  ``intrinsics``  ``{"color": {"fx", "fy", "cx", "cy", "width", "height", "model", "d"}}`` with
                  ``model`` one of :data:`DISTORTION_MODELS` and ``d`` its five coefficients
                  (k1, k2, p1, p2, k3; all zero for ``none``).

and each sample's ``timing``
  ``source_timestamp_ns``, ``clock_id`` and ``source_clock`` of the colour image;
  ``source_times``: ``{"rgb": {...}, "depth": {...}}`` each with ``source_timestamp_ns``, ``clock_id`` and
  ``source_clock`` on the Body host clock (``core.timing.host_clock_id``), and ``"pairing": {"valid": true}``
  once the driver has checked that both images come from the same capture;
  ``source_time_error``: set when the driver could not establish source times; the sample is refused.

Each driver owns the conversion from SDK timestamps to the declared source clock.
"""

from __future__ import annotations

from typing import Any

KIND = "camera"
STREAM = "rgbd"
REQUIRED_SIGNALS = ("rgb", "depth", "intrinsics")
COMPACT_DEPTH = "depth_png"
DISTORTION_MODELS = ("none", "brown_conrady", "inverse_brown_conrady")


def problems(device: dict[str, Any]) -> list[str]:
    """Every way one ``describe`` device entry falls short of the camera contract; empty when it conforms."""
    found = []
    if device.get("kind") != KIND:
        found.append(f"kind is {device.get('kind')!r}, not {KIND!r}")
    identity = (device.get("metadata") or {}).get("identity")
    if not isinstance(identity, dict) or not identity:
        found.append("metadata.identity must be a non-empty mapping")
    stream = (device.get("observation_streams") or {}).get(STREAM)
    if stream is None:
        found.append(f"observation stream {STREAM} is not declared")
    else:
        declared = stream.get("signals") or []
        found += [f"stream {STREAM} lacks signal {name}" for name in REQUIRED_SIGNALS if name not in declared]
    return found


def signals_to_read(device: dict[str, Any]) -> list[str]:
    """The ``rgbd`` signals the environment requests: the compact depth encoding when the driver offers it."""
    declared = ((device.get("observation_streams") or {}).get(STREAM) or {}).get("signals") or []
    return ["rgb", COMPACT_DEPTH if COMPACT_DEPTH in declared else "depth", "intrinsics"]
