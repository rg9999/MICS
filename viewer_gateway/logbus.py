"""/rosout aggregation -> bounded LogBatch channel (PRD viewer §6.2 #6, §7.1).

Kept on a separate channel from FrameSnapshot so a log burst can never stall
the 3D scene. Severity is filtered at the gateway; the buffer is bounded so
memory stays flat under a firehose.
"""

from __future__ import annotations

from collections import deque
from threading import Lock

from .aggregator import GwLog

# rcl_interfaces/msg/Log numeric levels -> grid labels
_LEVEL_NAMES = {10: "DEBUG", 20: "INFO", 30: "WARN", 40: "ERROR", 50: "FATAL"}


def level_name(level: int) -> str:
    # ROS levels are bucketed; round down to the nearest known severity
    for n in (50, 40, 30, 20, 10):
        if level >= n:
            return _LEVEL_NAMES[n]
    return "DEBUG"


class LogBus:
    def __init__(self, min_level: int = 20, ring_rows: int = 20000):
        self.min_level = min_level
        self._pending: deque[GwLog] = deque(maxlen=ring_rows)
        self._lock = Lock()

    def ingest(self, rec: GwLog):
        if rec.level < self.min_level:
            return
        with self._lock:
            self._pending.append(rec)

    def drain_batch(self) -> dict | None:
        with self._lock:
            if not self._pending:
                return None
            records = list(self._pending)
            self._pending.clear()
        return {
            "type": "logs",
            "records": [
                {"stamp": r.stamp, "level": level_name(r.level), "source": r.source,
                 "msg": r.msg, "file": r.file, "func": r.func, "line": r.line}
                for r in records
            ],
        }

    def clear(self):
        with self._lock:
            self._pending.clear()
