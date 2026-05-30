"""Safety node — PRD §6.5 / FR-OB-8.

Always-on, highest priority. Enforces:
  - hard geofence (auto-RTL on breach),
  - per-drone arm/disarm + global kill switch,
  - capture interlock: payload may NOT fire outside capture geometry or when a
    teammate is within capture_min_teammate_sep_m,
  - low-battery RTL.

The capture interlock is deliberately SEPARATE from mics_terminal so the fire
decision is double-gated (PRD risk table: 0 false-fire required).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .geometry import norm


@dataclass
class SafetyConfig:
    geofence_radius_m: float = 500.0
    geofence_center: np.ndarray = None
    capture_min_teammate_sep_m: float = 10.0
    low_battery_pct: float = 15.0

    def __post_init__(self) -> None:
        if self.geofence_center is None:
            self.geofence_center = np.zeros(3)


@dataclass
class Safety:
    cfg: SafetyConfig
    armed: bool = True
    killed: bool = False  # global kill from GS

    def arm(self) -> None:
        self.armed = True

    def disarm(self) -> None:
        self.armed = False

    def kill(self) -> None:
        self.killed = True

    def geofence_ok(self, pos: np.ndarray) -> bool:
        return norm(pos - self.cfg.geofence_center) <= self.cfg.geofence_radius_m

    def battery_ok(self, battery_pct: float) -> bool:
        return battery_pct > self.cfg.low_battery_pct

    def must_rtl(self, pos: np.ndarray, battery_pct: float) -> bool:
        """True if any safety condition forces a return-to-launch."""
        if self.killed:
            return True
        if not self.geofence_ok(pos):
            return True
        if not self.battery_ok(battery_pct):
            return True
        return False

    def capture_permitted(self, drone_pos: np.ndarray, geometry_ok: bool,
                          teammate_positions: list[np.ndarray]) -> bool:
        """Final interlock gate. ALL conditions must hold to permit fire."""
        if self.killed or not self.armed:
            return False
        if not geometry_ok:
            return False
        for tp in teammate_positions:
            if norm(drone_pos - tp) < self.cfg.capture_min_teammate_sep_m:
                return False
        return True
