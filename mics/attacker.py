"""Attacker (target) simulation — PRD §5.2 / FR-SIM-1.

Spawns target airframes and drives them along a selectable profile. Publishes
ground-truth state that is consumed ONLY by sensor models and scoring, never by
the autonomy directly (PRD §5.2).
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .geometry import clamp_norm, norm, unit


@dataclass
class AttackerState:
    target_id: int
    position: np.ndarray
    velocity: np.ndarray
    alive: bool = True


@dataclass
class Attacker:
    """A single scripted attacker.

    Profiles (PRD §5.2):
      waypoint — straight legs between points.
      ingress  — constant-heading run toward a defended asset.
      evasive  — random heading/altitude jinks when a pursuer is within r_evade.
      replay   — follow a recorded trajectory (list of positions).
    """

    target_id: int
    profile: str = "ingress"
    speed_mps: float = 15.0
    position: np.ndarray = field(default_factory=lambda: np.zeros(3))
    waypoints: list[np.ndarray] = field(default_factory=list)
    asset: np.ndarray = field(default_factory=lambda: np.zeros(3))
    r_evade_m: float = 80.0
    replay_path: list[np.ndarray] = field(default_factory=list)
    rng: np.random.Generator = field(default_factory=lambda: np.random.default_rng(0))

    # internal
    _heading: np.ndarray = field(default_factory=lambda: np.array([1.0, 0.0, 0.0]))
    _wp_idx: int = 0
    _replay_idx: int = 0
    _jink_timer: float = 0.0
    alive: bool = True

    def __post_init__(self) -> None:
        self.position = np.asarray(self.position, dtype=float)
        self.asset = np.asarray(self.asset, dtype=float)
        if self.profile == "ingress":
            self._heading = unit(self.asset - self.position)
            if norm(self._heading) < 1e-9:
                self._heading = np.array([1.0, 0.0, 0.0])

    def state(self) -> AttackerState:
        return AttackerState(self.target_id, self.position.copy(),
                             self._heading * self.speed_mps, self.alive)

    def step(self, dt: float, defenders_pos: list[np.ndarray]) -> None:
        if not self.alive:
            return
        if self.profile == "waypoint":
            self._step_waypoint(dt)
        elif self.profile == "ingress":
            self._step_ingress(dt)
        elif self.profile == "evasive":
            self._step_evasive(dt, defenders_pos)
        elif self.profile == "replay":
            self._step_replay(dt)
        else:
            raise ValueError(f"unknown attacker profile: {self.profile}")

    # --- profiles ---------------------------------------------------------

    def _advance(self, dt: float) -> None:
        self.position = self.position + self._heading * self.speed_mps * dt

    def _step_waypoint(self, dt: float) -> None:
        if not self.waypoints:
            self._advance(dt)
            return
        target = self.waypoints[self._wp_idx]
        to = target - self.position
        if norm(to) < self.speed_mps * dt + 0.5:
            self._wp_idx = (self._wp_idx + 1) % len(self.waypoints)
            target = self.waypoints[self._wp_idx]
            to = target - self.position
        self._heading = unit(to)
        self._advance(dt)

    def _step_ingress(self, dt: float) -> None:
        to = self.asset - self.position
        if norm(to) > 1e-6:
            self._heading = unit(to)
        self._advance(dt)

    def _step_evasive(self, dt: float, defenders_pos: list[np.ndarray]) -> None:
        threatened = any(norm(self.position - d) < self.r_evade_m for d in defenders_pos)
        # baseline ingress heading toward asset
        to_asset = self.asset - self.position
        base = unit(to_asset) if norm(to_asset) > 1e-6 else self._heading
        if threatened:
            self._jink_timer -= dt
            if self._jink_timer <= 0.0:
                # pick a new random jink offset every 0.5-1.5 s
                self._jink_timer = float(self.rng.uniform(0.5, 1.5))
                yaw = float(self.rng.uniform(-np.pi / 2, np.pi / 2))
                dz = float(self.rng.uniform(-0.4, 0.4))
                c, s = np.cos(yaw), np.sin(yaw)
                jinked = np.array([
                    base[0] * c - base[1] * s,
                    base[0] * s + base[1] * c,
                    base[2] + dz,
                ])
                self._heading = unit(jinked)
        else:
            self._heading = base
        self._advance(dt)

    def _step_replay(self, dt: float) -> None:
        if not self.replay_path:
            self._advance(dt)
            return
        if self._replay_idx < len(self.replay_path) - 1:
            self._replay_idx += 1
        prev = self.position.copy()
        self.position = np.asarray(self.replay_path[self._replay_idx], dtype=float)
        moved = self.position - prev
        if norm(moved) > 1e-9:
            self._heading = unit(moved)

    def kill(self) -> None:
        """Mark captured/removed (used by scoring on a successful capture)."""
        self.alive = False
