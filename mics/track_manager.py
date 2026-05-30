"""GS-side track manager — PRD §3.1 / §7.1.

Associates incoming partial tracks to existing GS tracks (nearest-neighbour
gating), ages out stale tracks after a TTL, and fuses repeated observations of
the same target into one smoothed track. Publishes the fused picture at >=5 Hz
(the sim loop drives the rate).
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .geometry import norm
from .msgs import TargetTrack, TrackSource


@dataclass
class _GsTrack:
    track: TargetTrack
    last_update: float


@dataclass
class TrackManager:
    ttl_s: float = 2.0
    assoc_gate_m: float = 50.0
    smoothing: float = 0.5  # 0=trust new fully, 1=trust prior fully

    _tracks: dict = field(default_factory=dict)  # target_id -> _GsTrack
    _next_id: int = 1

    def ingest(self, now: float, tracks: list[TargetTrack]) -> None:
        for t in tracks:
            self._associate_and_update(now, t)

    def _associate_and_update(self, now: float, t: TargetTrack) -> None:
        tid = t.target_id
        # if the source provided a target_id, trust it for association
        if tid in self._tracks:
            prev = self._tracks[tid].track
            a = self.smoothing
            fused_pos = a * prev.position + (1 - a) * t.position
            fused_vel = (a * prev.velocity + (1 - a) * t.velocity
                         if t.has_velocity else prev.velocity)
            t.position = fused_pos
            t.velocity = fused_vel
        else:
            # try spatial association to an existing track with a different id
            match = self._nearest(t.position)
            if match is not None:
                tid = match
                t.target_id = tid
        t.age = 0.0
        self._tracks[tid] = _GsTrack(track=t, last_update=now)

    def _nearest(self, pos: np.ndarray):
        best, best_d = None, self.assoc_gate_m
        for tid, gt in self._tracks.items():
            d = norm(gt.track.position - pos)
            if d < best_d:
                best, best_d = tid, d
        return best

    def step(self, now: float) -> None:
        """Age tracks and drop stale ones (FR-TGT-3)."""
        stale = []
        for tid, gt in self._tracks.items():
            gt.track.age = now - gt.last_update
            if gt.track.age > self.ttl_s:
                stale.append(tid)
        for tid in stale:
            del self._tracks[tid]

    def tracks(self) -> list[TargetTrack]:
        return [gt.track for gt in self._tracks.values()]

    def get(self, target_id: int) -> TargetTrack | None:
        gt = self._tracks.get(target_id)
        return gt.track if gt else None
