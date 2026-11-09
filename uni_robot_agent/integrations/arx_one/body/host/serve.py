"""Serve a Body: an installed instance from the host configuration, or one explicit driver for development."""

from __future__ import annotations

import argparse
import json
import signal
import threading
from contextlib import ExitStack
from pathlib import Path

from ..core.contract import Principal
from ..core.resources import runtime_root
from ..core.rpc import BodyHttpServer
from ..core.server import BodyServer
from .config import load_host
from .kit import factory, load_kit


def serve_instance(config, instance):
    from .system import load_tokens

    host = load_host(config)
    if instance not in host["bodies"]:
        raise ValueError(f"unknown Body instance {instance!r}; host.json has {sorted(host['bodies'])}")
    tokens = load_tokens(host["tokens"])
    spec = host["bodies"][instance]
    kit = load_kit(spec["kit"])
    added = kit.driver_args(host, instance)
    overridden = sorted(set(added) & set(spec["driver_args"]))
    if overridden or "body_id" in added:
        raise ValueError(f"kit {spec['kit']} must not override configured driver_args {overridden or ['body_id']}")
    stop = threading.Event()
    with ExitStack() as cleanup:
        # Signals only request shutdown, from the start: a SIGTERM while the driver opens
        # still runs every cleanup below. Handlers are restored last.
        for sig in (signal.SIGINT, signal.SIGTERM):
            previous = signal.signal(sig, lambda *_: stop.set())
            cleanup.callback(signal.signal, sig, previous)
        driver = factory(kit.factory)(**spec["driver_args"], **added, body_id=spec["body_id"])
        if callable(getattr(driver, "close", None)):
            cleanup.callback(driver.close)
        server = BodyServer(
            driver,
            tokens=tokens,
            journal_dir=Path(host["state_dir"]) / instance,
            lease_ttl_s=float(spec.get("lease_ttl_s", 5)),
        )
        cleanup.callback(server.close)
        manifest = runtime_root() / "release.json"
        release = (
            json.loads(manifest.read_text())
            if manifest.exists()
            else {"build_id": "development"}
        )
        server.runtime_info = {
            "host_id": host["host_id"],
            "instance": instance,
            "build_id": release["build_id"],
            "protocol_version": "2.0",
            "source_digest": release.get("source_digest"),
            "runtime_root": str(runtime_root()),
        }
        http = BodyHttpServer(
            server, host=host.get("listen", "127.0.0.1"), port=int(spec["port"])
        )
        cleanup.callback(http.stop)
        print(
            json.dumps(
                {
                    "body_id": spec["body_id"],
                    "host_id": host["host_id"],
                    "url": http.start(),
                    "build_id": release["build_id"],
                }
            ),
            flush=True,
        )
        stop.wait()


def serve_driver(args):
    tokens = {} if not args.tokens else {
        key: Principal.from_dict(value)
        for key, value in json.loads(Path(args.tokens).read_text()).items()
    }
    if args.host not in ("127.0.0.1", "localhost") and not tokens:
        raise ValueError("a token file is required for non-loopback binding")
    make_driver = factory(args.driver)
    stop = threading.Event()
    with ExitStack() as cleanup:
        # Signals only request shutdown. Never acquire locks or perform I/O in a handler.
        for sig in (signal.SIGINT, signal.SIGTERM):
            previous = signal.signal(sig, lambda *_: stop.set())
            cleanup.callback(signal.signal, sig, previous)
        driver = make_driver(**json.loads(args.driver_args))
        if callable(getattr(driver, "close", None)):
            cleanup.callback(driver.close)
        body = BodyServer(driver, tokens=tokens, lease_ttl_s=args.lease_ttl_s,
                          journal_dir=args.journal_dir)
        cleanup.callback(body.close)
        http = BodyHttpServer(body, host=args.host, port=args.port)
        cleanup.callback(http.stop)
        print(json.dumps({"url": http.start(), "body_id": body.manifest.body_id}), flush=True)
        stop.wait()
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    installed = parser.add_argument_group("installed instance (rua-host body)")
    installed.add_argument("--config", help="Host configuration")
    installed.add_argument("--instance", help="Body instance in the host configuration")
    development = parser.add_argument_group("one explicit driver (development)")
    development.add_argument("--driver", help="module:factory")
    development.add_argument("--driver-args", default="{}", help="JSON factory kwargs")
    development.add_argument("--host", default="127.0.0.1")
    development.add_argument("--port", type=int, default=0)
    development.add_argument("--tokens", help="JSON token-to-principal map")
    development.add_argument("--journal-dir", help="Stable per-Body directory for operation recovery")
    development.add_argument("--lease-ttl-s", type=float, default=5.)
    args = parser.parse_args(argv)
    if args.driver:
        if args.config or args.instance:
            parser.error("--driver serves one driver; do not combine it with --config or --instance")
        return serve_driver(args)
    if not (args.config and args.instance):
        parser.error("--config and --instance are required unless --driver is given")
    serve_instance(args.config, args.instance)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
