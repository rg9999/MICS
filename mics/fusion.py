"""Onboard multi-sensor fusion — PRD §3.4 / §6.3.

An EKF over a constant-velocity target model in the drone's LOCAL frame. State is
[px,py,pz,vx,vy,vz] (target position+velocity relative to the world, expressed in
local coordinates here for simplicity). Per-sensor measurement models:

  camera -> bearing + elevation (nonlinear)
  radar  -> range + range-rate + coarse azimuth (nonlinear)
  lidar  -> direct position (linear)

Each update runs validity gating (normalised innovation squared / Mahalanobis).
`quality` (0..1) is derived from covariance trace + recency and drives the
handoff and failure logic (FR-OB-3, FR-OB-6).
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .msgs import TargetEstimate
from .sensors import CameraMeas, LidarMeas, RadarMeas


def _f(state: np.ndarray, dt: float) -> np.ndarray:
    F = np.eye(6)
    F[0, 3] = F[1, 4] = F[2, 5] = dt
    return F @ state


def _F_jac(dt: float) -> np.ndarray:
    F = np.eye(6)
    F[0, 3] = F[1, 4] = F[2, 5] = dt
    return F


def _Q(dt: float, accel_sigma: float) -> np.ndarray:
    """Process noise from a constant-velocity model with white accel."""
    q = accel_sigma ** 2
    dt2 = dt * dt
    dt3 = dt2 * dt / 2.0
    dt4 = dt2 * dt2 / 4.0
    Q = np.zeros((6, 6))
    for i in range(3):
        Q[i, i] = dt4 * q
        Q[i, i + 3] = dt3 * q
        Q[i + 3, i] = dt3 * q
        Q[i + 3, i + 3] = dt2 * q
    return Q


@dataclass
class FusionEKF:
    accel_sigma: float = 8.0
    gate_chi2: float = 16.0           # ~ 3-sigma gate for 2-3 dof measurements
    quality_sigma_floor: float = 0.5  # m, position sigma giving quality ~1
    quality_sigma_ceil: float = 25.0  # m, position sigma giving quality ~0
    lost_timeout: float = 1.0         # s without any update -> not valid

    x: np.ndarray = field(default_factory=lambda: np.zeros(6))
    P: np.ndarray = field(default_factory=lambda: np.eye(6) * 1e4)
    initialized: bool = False
    last_update: float = -1e9
    _drone_pos: np.ndarray = field(default_factory=lambda: np.zeros(3))
    _drone_vel: np.ndarray = field(default_factory=lambda: np.zeros(3))

    # camera/radar measurement noise (std) — kept here so models stay analytic
    cam_sigma: float = 0.02
    radar_range_sigma: float = 2.0
    radar_rr_sigma: float = 0.6
    radar_az_sigma: float = 0.08
    lidar_sigma: float = 0.3

    def set_ownship(self, pos: np.ndarray, vel: np.ndarray) -> None:
        self._drone_pos = np.asarray(pos, dtype=float)
        self._drone_vel = np.asarray(vel, dtype=float)

    def predict(self, dt: float) -> None:
        if not self.initialized or dt <= 0:
            return
        self.x = _f(self.x, dt)
        F = _F_jac(dt)
        self.P = F @ self.P @ F.T + _Q(dt, self.accel_sigma)

    # --- measurement updates ---------------------------------------------

    def _seed_from_lidar(self, stamp: float, m: LidarMeas) -> None:
        self.x = np.zeros(6)
        self.x[:3] = self._drone_pos + m.position
        self.P = np.diag([4, 4, 4, 100, 100, 100]).astype(float)
        self.initialized = True
        self.last_update = stamp

    def _seed_from_radar(self, stamp: float, m: RadarMeas) -> None:
        # crude seed: place target along measured azimuth at measured range,
        # zero elevation. Large covariance — refined by subsequent updates.
        p = self._drone_pos + np.array([
            m.range_m * np.cos(m.azimuth),
            m.range_m * np.sin(m.azimuth),
            0.0,
        ])
        self.x = np.zeros(6)
        self.x[:3] = p
        self.P = np.diag([400, 400, 900, 100, 100, 100]).astype(float)
        self.initialized = True
        self.last_update = stamp

    def _update(self, z: np.ndarray, h: np.ndarray, H: np.ndarray,
                R: np.ndarray, stamp: float) -> bool:
        y = z - h
        S = H @ self.P @ H.T + R
        try:
            S_inv = np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return False
        nis = float(y.T @ S_inv @ y)
        if nis > self.gate_chi2:
            return False  # gated out
        K = self.P @ H.T @ S_inv
        self.x = self.x + K @ y
        I = np.eye(6)
        self.P = (I - K @ H) @ self.P
        self.last_update = stamp
        return True

    def update_lidar(self, m: LidarMeas) -> bool:
        if not self.initialized:
            self._seed_from_lidar(m.stamp, m)
            return True
        z = self._drone_pos + m.position
        h = self.x[:3]
        H = np.zeros((3, 6))
        H[0, 0] = H[1, 1] = H[2, 2] = 1.0
        R = np.eye(3) * (self.lidar_sigma ** 2)
        return self._update(z, h, H, R, m.stamp)

    def update_camera(self, m: CameraMeas) -> bool:
        if not self.initialized:
            return False  # camera alone can't initialise range
        rel = self.x[:3] - self._drone_pos
        rx, ry, rz = rel
        rng_xy2 = rx * rx + ry * ry
        rng_xy = np.sqrt(rng_xy2)
        if rng_xy < 1e-3:
            return False
        az = np.arctan2(ry, rx)
        el = np.arctan2(rz, rng_xy)
        h = np.array([az, el])
        z = np.array([m.azimuth, m.elevation])
        # wrap azimuth innovation
        z[0] = az + np.arctan2(np.sin(m.azimuth - az), np.cos(m.azimuth - az))
        H = np.zeros((2, 6))
        # d(az)/d(rel)
        H[0, 0] = -ry / rng_xy2
        H[0, 1] = rx / rng_xy2
        r2 = rng_xy2 + rz * rz
        H[1, 0] = -rx * rz / (r2 * rng_xy)
        H[1, 1] = -ry * rz / (r2 * rng_xy)
        H[1, 2] = rng_xy / r2
        R = np.eye(2) * (self.cam_sigma ** 2)
        return self._update(z, h, H, R, m.stamp)

    def update_radar(self, m: RadarMeas) -> bool:
        if not self.initialized:
            self._seed_from_radar(m.stamp, m)
            return True
        rel = self.x[:3] - self._drone_pos
        vel_rel = self.x[3:] - self._drone_vel
        rng = np.linalg.norm(rel)
        if rng < 1e-3:
            return False
        los = rel / rng
        range_rate = float(np.dot(vel_rel, los))
        az = np.arctan2(rel[1], rel[0])
        h = np.array([rng, range_rate, az])
        z = np.array([m.range_m, m.range_rate, m.azimuth])
        z[2] = az + np.arctan2(np.sin(m.azimuth - az), np.cos(m.azimuth - az))
        H = np.zeros((3, 6))
        # range wrt position
        H[0, :3] = los
        # range-rate wrt position and velocity (approx; drop position 2nd-order)
        H[1, :3] = (vel_rel - range_rate * los) / rng
        H[1, 3:] = los
        # azimuth wrt position
        rng_xy2 = rel[0] ** 2 + rel[1] ** 2
        H[2, 0] = -rel[1] / rng_xy2
        H[2, 1] = rel[0] / rng_xy2
        R = np.diag([self.radar_range_sigma ** 2,
                     self.radar_rr_sigma ** 2,
                     self.radar_az_sigma ** 2])
        return self._update(z, h, H, R, m.stamp)

    # --- output -----------------------------------------------------------

    def quality(self, now: float) -> float:
        if not self.initialized:
            return 0.0
        if now - self.last_update > self.lost_timeout:
            return 0.0
        pos_sigma = float(np.sqrt(np.trace(self.P[:3, :3]) / 3.0))
        lo, hi = self.quality_sigma_floor, self.quality_sigma_ceil
        q = (hi - pos_sigma) / (hi - lo)
        return float(np.clip(q, 0.0, 1.0))

    def estimate(self, now: float) -> TargetEstimate:
        return TargetEstimate(
            stamp=now,
            position=self.x[:3].copy(),
            velocity=self.x[3:].copy(),
            covariance=self.P.copy(),
            quality=self.quality(now),
            valid=self.initialized and (now - self.last_update <= self.lost_timeout),
        )
