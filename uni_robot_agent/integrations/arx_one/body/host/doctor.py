"""Offline installation checks; optional authenticated, read-only Body inspection."""

from __future__ import annotations

import argparse
import json
import math
import platform
from pathlib import Path

from ..core.resources import runtime_root
from .config import load_host
from .kit import load_kit


def diagnose(config, *, connect=False, token_file=None, timeout_s=3.):
    if not math.isfinite(timeout_s) or not 0 < timeout_s <= 30:
        raise ValueError("timeout_s must be finite and in (0,30]")
    host = load_host(config)
    root = runtime_root()
    kits = {name: load_kit(name) for name in host["kits"]}
    native, requirements = {}, {}
    for name, kit in kits.items():
        native.update({f"{name}.{key}": path.is_file() for key, path in kit.native_files(root).items()})
        requirements.update({f"{name}.{key}": bool(ok) for key, ok in kit.requirements().items()})
    result = {"host_id": host["host_id"], "config_valid": True,
              "platform": platform.system(), "machine": platform.machine(),
              "runtime_root": str(root), "native_files": native, "requirements": requirements,
              "hardware_accessed": False, "body_queries_requested": bool(connect),
              "physical_stop_guaranteed": False, "bodies": {}}
    result["installation_ready"] = (platform.system() == "Linux" and all(native.values())
                                    and all(requirements.values()))
    if not connect:
        return result
    if not token_file:
        raise ValueError("--connect requires --token-file containing one observe-capable token")
    token = Path(token_file).expanduser().read_text().strip()
    if not token:
        raise ValueError("empty diagnostic token file")
    from ..core.client import HttpDeviceClient
    listen = host.get("listen", "127.0.0.1")
    address = {"0.0.0.0": "127.0.0.1", "localhost": "127.0.0.1", "::": "[::1]"}.get(
        listen, f"[{listen}]" if ":" in listen else listen)
    result["body_queries_ok"] = True
    for name, body in host["bodies"].items():
        client = HttpDeviceClient(f"http://{address}:{body['port']}", token=token, timeout_s=timeout_s)
        try:
            health = client.health()
            if health.get("body_id") != body["body_id"] or health.get("runtime", {}).get("host_id") != host["host_id"]:
                raise ValueError("Body/host identity mismatch")
            manifest = client.describe()
            ids, signals = kits[body["kit"]].probe(body)
            actual = {d["device_id"] for d in manifest["devices"]}
            if not set(ids) <= actual:
                raise ValueError("configured device missing from Body manifest")
            result["bodies"][name] = {"ok": True, "health": health, "manifest": manifest,
                "check": client.check(ids),
                "state": client.read_state(ids, include=signals, max_age_s=.5, timeout_s=timeout_s)}
        except Exception as exc:
            result["body_queries_ok"] = False
            result["bodies"][name] = {"ok": False, "error": str(exc)}
        finally:
            client.close()
    # Driver-backed reads are explicit and can reach hardware; no control calls are made.
    result["hardware_accessed"] = True
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True)
    parser.add_argument("--connect", action="store_true", help="Read declared Body endpoints; never acquire control")
    parser.add_argument("--token-file", help="File containing a single observe-capable token")
    parser.add_argument("--timeout-s", type=float, default=3.)
    args = parser.parse_args(argv)
    result = diagnose(args.config, connect=args.connect, token_file=args.token_file, timeout_s=args.timeout_s)
    print(json.dumps(result, indent=2))
    return 0 if result["installation_ready"] and result.get("body_queries_ok", True) else 1


if __name__ == "__main__":
    raise SystemExit(main())
