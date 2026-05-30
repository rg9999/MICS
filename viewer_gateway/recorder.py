"""Gateway-owned recording subsystem (PRD viewer §6.6, FR-V-37..43).

A new timestamped session dir per Start/Stop. Two parallel streams written as
rolling, gzipped JSONL segments — the FrameSnapshot state stream and the
LogBatch stream (logs are always part of a recording). A manifest indexes the
segments by time range for fast seeking. Sessions are immutable once stopped.

JSONL is chosen so recorded messages are byte-identical to live ones, making
the replay path trivial (the server just re-emits them).
"""

from __future__ import annotations

import gzip
import json
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path

from .config import RecordingConfig


@dataclass
class _Segment:
    path: Path
    fh: object
    rows: int = 0
    bytes: int = 0
    start_stamp: float | None = None
    end_stamp: float = 0.0
    start_wall: float = 0.0

    def ref(self, session_dir: Path) -> dict:
        return {
            "file": str(self.path.relative_to(session_dir)).replace("\\", "/"),
            "startStamp": self.start_stamp if self.start_stamp is not None else 0.0,
            "endStamp": self.end_stamp,
            "rows": self.rows,
        }


class _Stream:
    """One rolling, gzipped JSONL stream (state or logs)."""

    def __init__(self, sub_dir: Path, prefix: str, cfg: RecordingConfig):
        self.dir = sub_dir
        self.prefix = prefix
        self.cfg = cfg
        self.dir.mkdir(parents=True, exist_ok=True)
        self._seq = 0
        self._seg: _Segment | None = None
        self.segments: list[dict] = []

    def _open(self):
        self._seq += 1
        ext = ".jsonl.gz" if self.cfg.compress else ".jsonl"
        path = self.dir / f"{self.prefix}-{self._seq:06d}{ext}"
        fh = gzip.open(path, "wt", encoding="utf-8") if self.cfg.compress \
            else open(path, "w", encoding="utf-8")
        self._seg = _Segment(path=path, fh=fh, start_wall=time.monotonic())

    def _should_rotate(self) -> bool:
        s = self._seg
        if s is None:
            return True
        if s.bytes >= self.cfg.segment_rotate_mb * 1_000_000:
            return True
        if (time.monotonic() - s.start_wall) >= self.cfg.segment_rotate_seconds:
            return True
        return False

    def _rotate(self, session_dir: Path):
        if self._seg is not None and self._seg.rows > 0:
            self._seg.fh.close()
            self.segments.append(self._seg.ref(session_dir))
        elif self._seg is not None:
            self._seg.fh.close()
            try:
                self._seg.path.unlink()  # drop empty segment
            except OSError:
                pass
        self._open()

    def write(self, msg: dict, stamp: float, session_dir: Path):
        if self._should_rotate():
            self._rotate(session_dir)
        line = json.dumps(msg, separators=(",", ":")) + "\n"
        self._seg.fh.write(line)
        self._seg.rows += 1
        self._seg.bytes += len(line)
        if self._seg.start_stamp is None:
            self._seg.start_stamp = stamp
        self._seg.end_stamp = stamp

    def close(self, session_dir: Path):
        if self._seg is not None:
            self._seg.fh.close()
            if self._seg.rows > 0:
                self.segments.append(self._seg.ref(session_dir))
            else:
                try:
                    self._seg.path.unlink()
                except OSError:
                    pass
            self._seg = None


class Recorder:
    """Owns at most one active session at a time."""

    def __init__(self, cfg: RecordingConfig, datum: dict):
        self.cfg = cfg
        self.datum = datum
        self.root = Path(cfg.dir)
        self.active = False
        self.session_id = ""
        self._session_dir: Path | None = None
        self._state: _Stream | None = None
        self._logs: _Stream | None = None
        self._started_at = 0.0
        self._scenario = ""
        self._drone_count = 0
        self._target_count = 0
        self._seq = 0

    def _new_session_id(self) -> str:
        self._seq += 1
        iso = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H-%M-%S")
        return self.cfg.session_name_template.format(iso=iso, seq=self._seq)

    def start(self, scenario: str = "") -> str:
        if self.active:
            return self.session_id
        self.session_id = self._new_session_id()
        self._session_dir = self.root / self.session_id
        self._session_dir.mkdir(parents=True, exist_ok=True)
        self._state = _Stream(self._session_dir / "state", "state", self.cfg)
        self._logs = _Stream(self._session_dir / "logs", "logs", self.cfg)
        self._started_at = time.time()
        self._scenario = scenario
        self._drone_count = 0
        self._target_count = 0
        self.active = True
        self._write_manifest(stopped=False)
        return self.session_id

    def write_state(self, snapshot: dict):
        if not self.active:
            return
        self._drone_count = max(self._drone_count, len(snapshot.get("drones", [])))
        self._target_count = max(self._target_count, len(snapshot.get("tracks", [])))
        self._state.write(snapshot, float(snapshot.get("stamp", 0.0)), self._session_dir)

    def write_logs(self, batch: dict):
        if not self.active or not self.cfg.include_logs:
            return
        records = batch.get("records", [])
        stamp = float(records[-1]["stamp"]) if records else 0.0
        self._logs.write(batch, stamp, self._session_dir)

    def stop(self) -> str:
        if not self.active:
            return ""
        sid = self.session_id
        self._state.close(self._session_dir)
        self._logs.close(self._session_dir)
        self._write_manifest(stopped=True)
        self.active = False
        return sid

    @property
    def status(self) -> dict:
        size = 0
        if self._session_dir and self._session_dir.is_dir():
            size = sum(f.stat().st_size for f in self._session_dir.rglob("*")
                       if f.is_file())
        return {
            "recording": self.active,
            "sessionId": self.session_id if self.active else "",
            "elapsedS": (time.time() - self._started_at) if self.active else 0.0,
            "sizeBytes": size,
        }

    def _write_manifest(self, stopped: bool):
        manifest = {
            "sessionId": self.session_id,
            "startedAt": self._started_at,
            "stoppedAt": time.time() if stopped else None,
            "durationS": (time.time() - self._started_at) if stopped else None,
            "scenario": self._scenario,
            "datum": self.datum,
            "droneCount": self._drone_count,
            "targetCount": self._target_count,
            "stateSegments": self._state.segments if self._state else [],
            "logSegments": self._logs.segments if self._logs else [],
            "hasRosbag2": False,
        }
        (self._session_dir / "manifest.json").write_text(
            json.dumps(manifest, indent=2))
