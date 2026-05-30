"""Per-drone sensor models — PRD §5.4 / FR-SENS-1..3, FR-SIM-3.

Each model derives a measurement from attacker ground truth, then adds realistic
noise, FOV gating, and dropout. Three `sensor_mode` levels (PRD §4.5):

  ideal — near-truth detections, tiny noise (CPU default, P1-P2).
  stub  — realistic noise + dropout, no rendering/inference (CPU fusion tests).
  full  — same analytic models here; in the real stack this is Gazebo render +
          YOLO. We treat full == stub for the pure-Python core but keep the hook.

Measurements are expressed in the drone's LOCAL frame (target position relative
to the drone). Camera yields bearing+elevation, radar yields range+range-rate+
coarse bearing, lidar yields direct 3D position at close range.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .geometry import bearing_elevation, norm, unit


@dataclass
class CameraMeas:
    stamp: float
    azimuth: float
    elevation: float
    class_confidence: float


@dataclass
class RadarMeas:
    stamp: float
    range_m: float
    range_rate: float
    azimuth: float  # coarse


@dataclass
class LidarMeas:
    stamp: float
    position: np.ndarray  # relative, drone-local


# noise presets per mode: (camera_ang_sigma_rad, radar_range_sigma, radar_rr_sigma,
#                          radar_az_sigma, lidar_pos_sigma, dropout_prob)
_PRESETS = {
    "ideal": (0.002, 0.3, 0.1, 0.01, 0.05, 0.0),
    "stub":  (0.015, 2.0, 0.5, 0.06, 0.20, 0.05),
    "full":  (0.015, 2.0, 0.5, 0.06, 0.20, 0.05),
}


@dataclass
class SensorSuite:
    """Camera + radar + lidar models for one drone."""

    mode: str = "ideal"
    camera_fov_rad: float = np.deg2rad(100.0)
    camera_range_m: float = 400.0
    radar_range_m: float = 800.0
    lidar_range_m: float = 40.0
    rng: np.random.Generator = None
    camera_enabled: bool = True
    radar_enabled: bool = True
    lidar_enabled: bool = True

    def __post_init__(self) -> None:
        if self.mode not in _PRESETS:
            raise ValueError(f"unknown sensor_mode: {self.mode}")
        if self.rng is None:
            self.rng = np.random.default_rng()

    def _p(self):
        return _PRESETS[self.mode]

    def _drop(self) -> bool:
        return float(self.rng.random()) < self._p()[5]

    def camera(self, stamp: float, drone_pos: np.ndarray, drone_heading: np.ndarray,
               target_pos: np.ndarray) -> CameraMeas | None:
        if not self.camera_enabled or self._drop():
            return None
        rel = target_pos - drone_pos
        d = norm(rel)
        if d > self.camera_range_m or d < 1e-6:
            return None
        # FOV gating: target must be within half-FOV of the drone heading
        from .geometry import angle_between
        if angle_between(rel, drone_heading) > self.camera_fov_rad / 2.0:
            return None
        az, el = bearing_elevation(rel)
        s = self._p()[0]
        az += float(self.rng.normal(0, s))
        el += float(self.rng.normal(0, s))
        # confidence falls off with range
        conf = float(np.clip(1.0 - d / self.camera_range_m, 0.3, 0.99))
        return CameraMeas(stamp, az, el, conf)

    def radar(self, stamp: float, drone_pos: np.ndarray, drone_vel: np.ndarray,
              target_pos: np.ndarray, target_vel: np.ndarray) -> RadarMeas | None:
        # radar is the all-weather sensor: it ignores camera glare/darkness and
        # does NOT respect the narrow camera FOV (wide beam). Still has dropout.
        if not self.radar_enabled or self._drop():
            return None
        rel = target_pos - drone_pos
        d = norm(rel)
        if d > self.radar_range_m or d < 1e-6:
            return None
        rel_vel = target_vel - drone_vel
        los = unit(rel)
        range_rate = float(np.dot(rel_vel, los))
        az, _ = bearing_elevation(rel)
        _, rng_s, rr_s, az_s, _, _ = self._p()
        return RadarMeas(
            stamp,
            d + float(self.rng.normal(0, rng_s)),
            range_rate + float(self.rng.normal(0, rr_s)),
            az + float(self.rng.normal(0, az_s)),
        )

    def lidar(self, stamp: float, drone_pos: np.ndarray,
              target_pos: np.ndarray) -> LidarMeas | None:
        if not self.lidar_enabled or self._drop():
            return None
        rel = target_pos - drone_pos
        d = norm(rel)
        if d > self.lidar_range_m or d < 1e-6:
            return None
        s = self._p()[4]
        noisy = rel + self.rng.normal(0, s, size=3)
        return LidarMeas(stamp, noisy)
