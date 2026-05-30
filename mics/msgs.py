"""Message schemas (PRD §8) as plain dataclasses.

These mirror the ROS2 `mics_msgs` definitions. In the real stack they become
.msg files; here they are dataclasses so the same field semantics drive the
Python simulation. Vectors are numpy arrays in a shared global frame (ENU, metres)
unless noted as a drone-local frame.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import IntEnum

import numpy as np


class DroneState(IntEnum):
    """Onboard state machine (PRD §6.2)."""

    IDLE = 0
    ASSIGNED = 1
    MIDCOURSE = 2
    ACQUIRING = 3
    TERMINAL = 4
    CAPTURED = 5
    FAILED = 6


class TrackSource(IntEnum):
    INTERNAL_SIM = 0
    EXTERNAL = 1
    ONBOARD = 2


class CaptureResult(IntEnum):
    ATTEMPT = 0
    SUCCESS = 1
    MISS = 2


class AssignmentRole(IntEnum):
    PRIMARY = 0
    STANDBY = 1


def _zeros3() -> np.ndarray:
    return np.zeros(3, dtype=float)


@dataclass
class TargetTrack:
    """PRD §8.1 — a GS-side (cued, uncertain) target track."""

    stamp: float
    target_id: int
    position: np.ndarray = field(default_factory=_zeros3)
    velocity: np.ndarray = field(default_factory=_zeros3)
    position_covariance: np.ndarray = field(default_factory=lambda: np.eye(3))
    class_confidence: float = 0.0
    source: TrackSource = TrackSource.INTERNAL_SIM
    age: float = 0.0
    has_velocity: bool = True

    def pos_sigma(self) -> float:
        """Representative 1-sigma position uncertainty (m)."""
        return float(np.sqrt(np.trace(self.position_covariance) / 3.0))


@dataclass
class Assignment:
    """PRD §8.2 — a (drone -> target) mapping. target_id == 0 means unassigned."""

    stamp: float
    drone_id: int
    target_id: int = 0
    role: AssignmentRole = AssignmentRole.PRIMARY


@dataclass
class DroneStatus:
    """PRD §8.3 — broadcast on /state_sharing_drone_N."""

    stamp: float
    drone_id: int
    position: np.ndarray = field(default_factory=_zeros3)
    velocity: np.ndarray = field(default_factory=_zeros3)
    state: DroneState = DroneState.IDLE
    current_target: int = 0
    battery_pct: float = 100.0
    track_quality: float = 0.0


@dataclass
class CaptureEvent:
    """PRD §8.4 — emitted by mics_terminal on a capture attempt."""

    stamp: float
    drone_id: int
    target_id: int
    result: CaptureResult
    engagement_point: np.ndarray = field(default_factory=_zeros3)


@dataclass
class TargetEstimate:
    """Onboard fused estimate (PRD §6.3), in the drone's local frame.

    `quality` (0..1) drives handoff and failure logic.
    """

    stamp: float
    position: np.ndarray = field(default_factory=_zeros3)
    velocity: np.ndarray = field(default_factory=_zeros3)
    covariance: np.ndarray = field(default_factory=lambda: np.eye(6))
    quality: float = 0.0
    valid: bool = False
