"""Host configuration (host.json): the host, its Body instances and the kit that drives each; all paths explicit."""

from __future__ import annotations

import json
import math
import re
from pathlib import Path

from .kit import load_kit

SCHEMA_VERSION = 2
NAME = r"[A-Za-z0-9_-]+"


def load_host(path):
    path = Path(path).expanduser().resolve()
    data = json.loads(path.read_text())
    if data.get("schema_version") != SCHEMA_VERSION:
        raise ValueError(f"host schema_version must be {SCHEMA_VERSION}")
    missing = sorted({"host_id", "operator", "state_dir", "tokens", "kits", "bodies"} - set(data))
    if missing:
        raise ValueError(f"host.json is missing {missing}")
    if not re.fullmatch(NAME, data.get("host_id", "")):
        raise ValueError("invalid host_id")
    if not isinstance(data.get("bodies"), dict) or not data["bodies"]:
        raise ValueError("a nonempty bodies map is required")
    kits = data.get("kits")
    if not isinstance(kits, dict) or not all(isinstance(value, dict) for value in kits.values()):
        raise ValueError("kits must map each kit name to its configuration")
    ports, body_ids, resources = set(), set(), {}
    for name, body in data["bodies"].items():
        if not re.fullmatch(NAME, name):
            raise ValueError("invalid Body instance name")
        if not isinstance(body, dict) or {"kit", "body_id", "port"} - set(body):
            raise ValueError(f"{name}: a Body needs kit, body_id and port")
        if body.get("kit") not in kits:
            raise ValueError(f"{name}: kit {body.get('kit')!r} is not configured under kits")
        if body["body_id"] in body_ids:
            raise ValueError("duplicate body_id")
        body_ids.add(body["body_id"])
        port = body["port"]
        if type(port) is not int or not 1024 <= port <= 65535 or port in ports:
            raise ValueError("duplicate or invalid port")
        ports.add(port)
        ttl = float(body.get("lease_ttl_s", 5))
        if not math.isfinite(ttl) or ttl <= 0:
            raise ValueError("lease_ttl_s must be positive and finite")
        if not isinstance(body.setdefault("driver_args", {}), dict):
            raise ValueError(f"{name}: driver_args must be an object")
    for kit_name, config in kits.items():
        bodies = {name: body for name, body in data["bodies"].items() if body["kit"] == kit_name}
        kit = load_kit(kit_name)
        kit.validate(data, config, bodies)
        for name, body in bodies.items():
            for resource in kit.resources(body):
                if resource in resources:
                    raise ValueError(f"{name} and {resources[resource]} both claim {resource}")
                resources[resource] = name
    for field in ("state_dir", "tokens"):
        value = Path(data[field]).expanduser()
        if not value.is_absolute():
            value = path.parent / value
        data[field] = str(value.resolve())
    data["_path"] = str(path)
    return data
