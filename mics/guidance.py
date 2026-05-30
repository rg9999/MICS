"""Terminal pursuit guidance — PRD §6.4 / FR-OB-4.

Proportional navigation (PN): commanded lateral acceleration is proportional to
the line-of-sight (LOS) rotation rate and closing speed:

    a_cmd = N * Vc * omega_los

We produce a velocity setpoint (the offboard_control PN mode in the real stack
converts this to attitude/throttle). Output magnitude is capped at max_speed.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .geometry import clamp_norm, norm, unit


@dataclass
class ProportionalNavigation:
    nav_constant: float = 3.0
    max_speed: float = 25.0
    max_accel: float = 30.0

    def command(self, drone_pos: np.ndarray, drone_vel: np.ndarray,
                target_pos: np.ndarray, target_vel: np.ndarray,
                dt: float) -> np.ndarray:
        """Return a velocity setpoint (m/s) in the world frame."""
        rel = target_pos - drone_pos
        r = norm(rel)
        if r < 1e-6:
            return drone_vel
        los = rel / r
        rel_vel = target_vel - drone_vel
        closing_speed = -float(np.dot(rel_vel, los))  # positive when closing

        # LOS rotation rate vector: omega = (rel x rel_vel) / r^2
        omega = np.cross(rel, rel_vel) / (r * r)

        # PN acceleration command perpendicular to LOS
        vc = max(closing_speed, 1.0)
        a_cmd = self.nav_constant * vc * np.cross(omega, los)
        a_cmd = clamp_norm(a_cmd, self.max_accel)

        # Base velocity: head along LOS at max speed, plus PN correction.
        desired = los * self.max_speed + a_cmd * dt
        return clamp_norm(desired, self.max_speed)

    @staticmethod
    def closing_speed(drone_pos, drone_vel, target_pos, target_vel) -> float:
        rel = target_pos - drone_pos
        r = norm(rel)
        if r < 1e-6:
            return 0.0
        los = rel / r
        return -float(np.dot(target_vel - drone_vel, los))
