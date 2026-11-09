"""Durable immutable observation inputs with transactional status and events.

Adapted from the independent real-robot observation archive. No Agent, model,
platform, or device connection is required to reopen an archive.
"""
from __future__ import annotations

from contextlib import contextmanager
import hashlib
import io
import json
import os
from pathlib import Path
import sqlite3
import time
from uuid import uuid4

import numpy as np

from .snapshot import RGBDSnapshot


def encode(value) -> str:
    return json.dumps(value, allow_nan=False, sort_keys=True, separators=(",", ":"))


class ObservationStore:
    """One durable namespace per directory; never resume capture on reopen.

    SQLite serializes writers across threads and processes. Files are fsynced
    before publishing their hashes in the same transaction as the ready event.
    An interrupted write may leave an unreferenced file, never a ready record.
    """

    def __init__(self, root):
        self.root = Path(root).resolve() / "observations"
        self.root.mkdir(parents=True, exist_ok=True)
        with self._db() as db:
            db.executescript("""
                CREATE TABLE IF NOT EXISTS meta (key TEXT PRIMARY KEY, value TEXT NOT NULL);
                CREATE TABLE IF NOT EXISTS observations (
                    seq INTEGER PRIMARY KEY AUTOINCREMENT,
                    observe_id TEXT UNIQUE, request TEXT NOT NULL,
                    state TEXT NOT NULL, raw TEXT, result TEXT);
                CREATE TABLE IF NOT EXISTS events (
                    seq INTEGER PRIMARY KEY AUTOINCREMENT,
                    observe_id TEXT NOT NULL, event TEXT NOT NULL);
                CREATE TABLE IF NOT EXISTS derived (
                    artifact_id TEXT PRIMARY KEY, observe_id TEXT NOT NULL,
                    camera TEXT NOT NULL, record TEXT NOT NULL);
            """)
            db.execute("INSERT OR IGNORE INTO meta VALUES ('schema_version', '1')")
            version = db.execute("SELECT value FROM meta WHERE key='schema_version'").fetchone()[0]
            if version != "1":
                raise ValueError("unsupported observation archive version")
            db.execute("INSERT OR IGNORE INTO meta VALUES ('run_id', ?)", (uuid4().hex,))
            self.run_id = db.execute("SELECT value FROM meta WHERE key='run_id'").fetchone()[0]

    @contextmanager
    def _db(self, *, write=False):
        db = sqlite3.connect(self.root / "index.sqlite3", timeout=10)
        db.row_factory = sqlite3.Row
        db.execute("PRAGMA synchronous=FULL")
        try:
            if write:
                db.execute("BEGIN IMMEDIATE")
            with db:
                yield db
        finally:
            db.close()

    @staticmethod
    def _event(db, observe_id, phase, **data):
        record = {"phase": phase, "time_ns": str(time.time_ns()), **data}
        db.execute("INSERT INTO events(observe_id,event) VALUES (?,?)", (observe_id, encode(record)))

    @staticmethod
    def _row(db, observe_id):
        row = db.execute("SELECT * FROM observations WHERE observe_id=?", (observe_id,)).fetchone()
        if row is None:
            raise ValueError("unknown observe_id in this archive")
        return row

    def begin(self, request: dict) -> str:
        request_json = encode(request)
        with self._db(write=True) as db:
            seq = db.execute("INSERT INTO observations(request,state) VALUES (?, 'received')", (request_json,)).lastrowid
            observe_id = f"{self.run_id}/observe-{seq:06d}"
            db.execute("UPDATE observations SET observe_id=? WHERE seq=?", (observe_id, seq))
            self._event(db, observe_id, "received")
        return observe_id

    def pin(self, observe_id, snapshots: dict[str, RGBDSnapshot], state: dict) -> None:
        """Freeze the original pixels, timing, state and calibration evidence once."""
        encode(state)
        with self._db(write=True) as db:
            row = self._row(db, observe_id)
            if row["state"] != "received" or row["raw"] is not None:
                raise ValueError("observation raw input is already committed or terminal")
            cameras = {}
            for camera, snapshot in snapshots.items():
                buffer = io.BytesIO()
                np.savez(buffer, jpeg=np.frombuffer(snapshot.jpeg, dtype=np.uint8),
                         depth_mm=snapshot.depth_mm,
                         intrinsics=encode(snapshot.intrinsics), metadata=encode(snapshot.metadata))
                payload = buffer.getvalue()
                path = self.root / (uuid4().hex + ".npz")
                with path.open("xb") as out:
                    out.write(payload)
                    out.flush()
                    os.fsync(out.fileno())
                cameras[camera] = {"file": path.name, "sha256": hashlib.sha256(payload).hexdigest()}
            fd = os.open(self.root, os.O_RDONLY)
            try:
                os.fsync(fd)
            finally:
                os.close(fd)
            raw = {"state": state, "cameras": cameras, "committed_ns": str(time.time_ns())}
            db.execute("UPDATE observations SET raw=?,state='raw_ready' WHERE observe_id=?", (encode(raw), observe_id))
            self._event(db, observe_id, "raw_ready", cameras=list(cameras))

    def finish(self, observe_id, result: dict, *, state="completed") -> None:
        if state not in {"completed", "partial", "failed", "cancelled"}:
            raise ValueError("invalid terminal observation state")
        result_json = encode(result)
        with self._db(write=True) as db:
            row = self._row(db, observe_id)
            if row["state"] not in {"received", "raw_ready"}:
                raise ValueError("observation is already terminal")
            if state in {"completed", "partial"} and row["raw"] is None:
                raise ValueError("successful observations require committed raw input")
            db.execute("UPDATE observations SET state=?,result=? WHERE observe_id=?", (state, result_json, observe_id))
            self._event(db, observe_id, state)

    def get(self, observe_id, *, include_raw=True) -> dict:
        with self._db() as db:
            row = self._row(db, observe_id)
        result = {"schema_version": 1, "observe_id": observe_id, "observe_seq": row["seq"],
                  "request": json.loads(row["request"]), "state": row["state"],
                  "result": json.loads(row["result"]) if row["result"] else None}
        if include_raw:
            result["raw"] = json.loads(row["raw"]) if row["raw"] else None
        return result

    def list(self, *, limit=20, before_seq=None) -> list[dict]:
        if type(limit) is not int or not 1 <= limit <= 100:
            raise ValueError("limit must be an integer in 1..100")
        if before_seq is not None and (type(before_seq) is not int or before_seq <= 0):
            raise ValueError("before_seq must be a positive integer")
        with self._db() as db:
            rows = db.execute("SELECT observe_id,seq,state FROM observations WHERE seq<? ORDER BY seq DESC LIMIT ?",
                              (before_seq if before_seq is not None else 2**63-1, limit)).fetchall()
        return [{"observe_id": r["observe_id"], "observe_seq": r["seq"], "state": r["state"]} for r in rows]

    def events(self, *, after=0, limit=100) -> list[dict]:
        if type(after) is not int or after < 0 or type(limit) is not int or not 1 <= limit <= 1000:
            raise ValueError("invalid event cursor or limit")
        with self._db() as db:
            rows = db.execute("SELECT * FROM events WHERE seq>? ORDER BY seq LIMIT ?", (after, limit)).fetchall()
        return [{"cursor": r["seq"], "observe_id": r["observe_id"], **json.loads(r["event"])} for r in rows]

    def snapshot(self, observe_id, camera: str) -> RGBDSnapshot:
        raw = self.get(observe_id)["raw"]
        if not raw or camera not in raw["cameras"]:
            raise ValueError("observation has no snapshot for that camera")
        artifact = raw["cameras"][camera]
        path = (self.root / artifact["file"]).resolve()
        if path.parent != self.root:
            raise ValueError("invalid artifact path")
        if not path.exists():
            raise ValueError("observation bytes were not retained by this run's archive policy")
        payload = path.read_bytes()
        if hashlib.sha256(payload).hexdigest() != artifact["sha256"]:
            raise ValueError("observation artifact checksum mismatch")
        with np.load(io.BytesIO(payload), allow_pickle=False) as data:
            metadata = json.loads(str(data["metadata"]))
            # The archive is evidence, never a fresh motion precondition.
            metadata.update(archived=True, geometry_current=False)
            return RGBDSnapshot(data["jpeg"].tobytes(), data["depth_mm"].copy(),
                                json.loads(str(data["intrinsics"])), metadata, time.monotonic())

    def put_derived(self, observe_id, camera, *, kind, arrays, metadata) -> dict:
        """Append a checksummed result without changing the original observation.

        The raw artifact hash binds each result to one exact camera input. Failed
        writes may leave orphan files but cannot publish a partially written result.
        """
        if not isinstance(kind, str) or not kind.strip():
            raise ValueError("derived kind must be nonempty")
        if not isinstance(arrays, dict) or not arrays:
            raise ValueError("derived arrays must be a nonempty mapping")
        copied = {}
        for key, value in arrays.items():
            array = np.asarray(value)
            if not isinstance(key, str) or not key.isidentifier() or key == "file" or array.dtype.kind not in "buif" or not np.isfinite(array).all():
                raise ValueError("derived arrays need simple names and finite numeric values")
            copied[key] = array.copy()
        # Verify the original bytes before publishing their identity.
        self.snapshot(observe_id, camera)
        metadata = json.loads(encode(metadata))
        with self._db(write=True) as db:
            row = self._row(db, observe_id)
            raw = json.loads(row["raw"])
            artifact_id = uuid4().hex
            buffer = io.BytesIO()
            np.savez(buffer, **copied)
            payload = buffer.getvalue()
            filename = artifact_id + ".npz"
            with (self.root / filename).open("xb") as out:
                out.write(payload)
                out.flush()
                os.fsync(out.fileno())
            fd = os.open(self.root, os.O_RDONLY)
            try:
                os.fsync(fd)
            finally:
                os.close(fd)
            record = {"artifact_id": artifact_id, "observe_id": observe_id,
                      "camera": camera, "kind": kind, "metadata": metadata,
                      "raw_sha256": raw["cameras"][camera]["sha256"],
                      "file": filename, "sha256": hashlib.sha256(payload).hexdigest()}
            db.execute("INSERT INTO derived VALUES (?,?,?,?)", (artifact_id, observe_id, camera, encode(record)))
            self._event(db, observe_id, "derived_ready", artifact_id=artifact_id, camera=camera, kind=kind)
        return record

    def derived(self, observe_id, camera, artifact_id) -> tuple[dict, dict]:
        """Read one explicit result; never select the latest or another camera."""
        with self._db() as db:
            row = db.execute("SELECT record FROM derived WHERE artifact_id=? AND observe_id=? AND camera=?",
                             (artifact_id, observe_id, camera)).fetchone()
        if row is None:
            raise ValueError("derived artifact does not belong to this observation and camera")
        record = json.loads(row["record"])
        self.snapshot(observe_id, camera)
        raw = self.get(observe_id)["raw"]["cameras"][camera]
        if record["raw_sha256"] != raw["sha256"]:
            raise ValueError("derived artifact raw input mismatch")
        path = (self.root / record["file"]).resolve()
        if path.parent != self.root:
            raise ValueError("invalid derived artifact path")
        payload = path.read_bytes()
        if hashlib.sha256(payload).hexdigest() != record["sha256"]:
            raise ValueError("derived artifact checksum mismatch")
        with np.load(io.BytesIO(payload), allow_pickle=False) as data:
            arrays = {key: data[key].copy() for key in data.files}
        return record, arrays

    def processing_event(self, observe_id, phase, **data):
        """Append processing status without changing immutable capture status."""
        with self._db(write=True) as db:
            self._row(db, observe_id)
            self._event(db, observe_id, phase, **data)

    def derived_records(self, observe_id):
        with self._db() as db:
            self._row(db, observe_id)
            rows = db.execute("SELECT record FROM derived WHERE observe_id=? ORDER BY rowid", (observe_id,)).fetchall()
        return [json.loads(row["record"]) for row in rows]
