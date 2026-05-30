"""Cue degradation — PRD §5.5 / FR-SIM-4.

Converts attacker ground truth into PARTIAL GS tracks: Gaussian position noise,
sometimes-absent velocity, update dropout, latency, and optional ghost tracks.
This guarantees interceptors are tested against imperfect cues, matching the real
external-source case. Lives in mics_target_ingest when target_source=internal.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field

import numpy as np

from .attacker import AttackerState
from .msgs import TargetTrack, TrackSource


@dataclass
class CueDegrader:
    pos_sigma_m: float = 15.0
    dropout_pct: float = 20.0
    latency_ms: float = 300.0
    vel_dropout_pct: float = 30.0
    ghost_rate_hz: float = 0.0
    rng: np.random.Generator = None

    # latency buffer of (release_time, TargetTrack)
    _pending: deque = field(default_factory=deque)
    _last_real: dict = field(default_factory=dict)

    def __post_init__(self) -> None:
        if self.rng is None:
            self.rng = np.random.default_rng()

    def ingest(self, now: float, truth: list[AttackerState]) -> None:
        """Sample truth into the latency buffer, applying dropout + noise."""
        latency = self.latency_ms / 1000.0
        for st in truth:
            if not st.alive:
                continue
            if float(self.rng.random()) < self.dropout_pct / 100.0:
                continue  # dropped update
            cov = np.eye(3) * (self.pos_sigma_m ** 2)
            pos = st.position + self.rng.normal(0, self.pos_sigma_m, size=3)
            has_vel = float(self.rng.random()) >= self.vel_dropout_pct / 100.0
            vel = st.velocity.copy() if has_vel else np.zeros(3)
            trk = TargetTrack(
                stamp=now,
                target_id=st.target_id,
                position=pos,
                velocity=vel,
                position_covariance=cov,
                class_confidence=float(self.rng.uniform(0.6, 0.95)),
                source=TrackSource.INTERNAL_SIM,
                age=0.0,
                has_velocity=has_vel,
            )
            self._pending.append((now + latency, trk))

    def release(self, now: float) -> list[TargetTrack]:
        """Return tracks whose latency window has elapsed."""
        out: list[TargetTrack] = []
        while self._pending and self._pending[0][0] <= now:
            _, trk = self._pending.popleft()
            self._last_real[trk.target_id] = now
            out.append(trk)
        return out
