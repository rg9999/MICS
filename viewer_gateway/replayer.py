"""Replay = the gateway re-emitting its own recorded JSONL stream over the same
socket (PRD viewer §6.5). Because recorded messages are byte-identical to live
ones, the client reconstructs scene + grids + logs with no special-casing.
"""

from __future__ import annotations

import gzip
import json
from pathlib import Path


def list_recordings(recordings_dir: str | Path) -> list[dict]:
    """All session manifests under the recordings root (FR-V-41)."""
    root = Path(recordings_dir)
    if not root.is_dir():
        return []
    out = []
    for d in sorted(root.iterdir()):
        mf = d / "manifest.json"
        if mf.is_file():
            try:
                out.append(json.loads(mf.read_text()))
            except (OSError, json.JSONDecodeError):
                continue
    return out


def load_manifest(session_dir: str | Path) -> dict:
    return json.loads((Path(session_dir) / "manifest.json").read_text())


def _open(path: Path):
    if path.suffix == ".gz":
        return gzip.open(path, "rt", encoding="utf-8")
    return open(path, "r", encoding="utf-8")


def _iter_segments(session_dir: Path, segments: list[dict]):
    for seg in segments:
        path = session_dir / seg["file"]
        if not path.is_file():
            continue
        with _open(path) as fh:
            for line in fh:
                line = line.strip()
                if line:
                    yield json.loads(line)


def iter_state(session_dir: str | Path):
    d = Path(session_dir)
    yield from _iter_segments(d, load_manifest(d).get("stateSegments", []))


def iter_logs(session_dir: str | Path):
    d = Path(session_dir)
    yield from _iter_segments(d, load_manifest(d).get("logSegments", []))


def iter_merged(session_dir: str | Path):
    """Yield (stamp, message) for state + logs merged in stamp order.

    Drives the single live/replay code path: the server paces these to the
    Cesium clock (scrub/pause/speed) and pushes each message over the socket.
    """
    d = Path(session_dir)
    events: list[tuple[float, dict]] = []
    for snap in iter_state(d):
        events.append((float(snap.get("stamp", 0.0)), snap))
    for batch in iter_logs(d):
        recs = batch.get("records", [])
        stamp = float(recs[-1]["stamp"]) if recs else 0.0
        events.append((stamp, batch))
    events.sort(key=lambda e: e[0])
    yield from events
