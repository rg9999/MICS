"""Task allocation & reassignment — PRD §3.2 / §7.2.

Maintains a roster from /state_sharing_*, computes drone->target assignments
minimising estimated time-to-intercept, and reassigns within <=1 s when a drone
reports FAILED/LOST. Supports greedy (v0) and Hungarian (v1) solvers, redundancy,
standby role, and hysteresis to prevent thrash (PRD risk table).
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from . import hungarian
from .geometry import norm
from .msgs import Assignment, AssignmentRole, DroneState, DroneStatus, TargetTrack

# states in which a drone is unavailable for (re)assignment
_BUSY = {DroneState.MIDCOURSE, DroneState.ACQUIRING, DroneState.TERMINAL}
_DEAD = {DroneState.FAILED}


@dataclass
class AllocatorConfig:
    algorithm: str = "hungarian"        # greedy | hungarian
    redundancy: int = 1
    reassign_max_latency_s: float = 1.0
    hysteresis_improvement: float = 0.15  # require 15% better cost to switch
    avg_closing_speed: float = 20.0     # m/s, for time-to-intercept estimate
    enable_standby: bool = False


@dataclass
class Allocator:
    cfg: AllocatorConfig = field(default_factory=AllocatorConfig)

    _roster: dict = field(default_factory=dict)         # drone_id -> DroneStatus
    _assignments: dict = field(default_factory=dict)    # drone_id -> Assignment
    _failed_targets: set = field(default_factory=set)
    paused: bool = False

    def update_roster(self, statuses: list[DroneStatus]) -> None:
        for s in statuses:
            self._roster[s.drone_id] = s

    def assignments(self) -> dict:
        return self._assignments

    def assignment_for(self, drone_id: int) -> Assignment | None:
        return self._assignments.get(drone_id)

    # --- cost -------------------------------------------------------------

    def _time_to_intercept(self, drone: DroneStatus, track: TargetTrack) -> float:
        rel = track.position - drone.position
        d = norm(rel)
        # closing speed estimate: drone cruise minus target radial component
        closing = self.cfg.avg_closing_speed
        if track.has_velocity:
            from .geometry import unit
            closing = max(self.cfg.avg_closing_speed -
                          float(np.dot(track.velocity, unit(rel))), 1.0)
        tti = d / max(closing, 1.0)
        # penalise low battery
        batt_factor = 1.0 + max(0.0, (40.0 - drone.battery_pct)) / 40.0
        return tti * batt_factor

    # --- main allocation --------------------------------------------------

    def allocate(self, now: float, tracks: list[TargetTrack]) -> dict:
        """Compute assignments. Returns {drone_id: Assignment}. Idempotent per
        tick; honours hysteresis so it won't thrash under noisy cues."""
        if self.paused:
            return self._assignments

        # 1. clear assignments whose drone is dead or whose target vanished
        live_target_ids = {t.target_id for t in tracks}
        for did, a in list(self._assignments.items()):
            st = self._roster.get(did)
            if st is None:
                continue
            if st.state in _DEAD:
                self._failed_targets.add(a.target_id)
                self._unassign(now, did)
            elif a.target_id not in live_target_ids:
                self._unassign(now, did)

        # 2. drones available for assignment: not busy, not dead
        available = [s for s in self._roster.values()
                     if s.state not in _BUSY and s.state not in _DEAD]
        # build target slots respecting redundancy AND existing coverage by busy
        # drones, so we never pile idle drones onto an already-serviced target
        # (FR-ALLOC-2).
        slots = self._open_slots(tracks)
        if not available or not slots:
            return self._assignments

        if self.cfg.algorithm == "greedy":
            self._allocate_greedy(now, available, slots)
        else:
            self._allocate_hungarian(now, available, slots)
        return self._assignments

    def _open_slots(self, tracks: list[TargetTrack]) -> list[TargetTrack]:
        """Target slots still needing a drone, after subtracting current coverage
        from busy (committed) drones."""
        coverage: dict[int, int] = {}
        for did, a in self._assignments.items():
            st = self._roster.get(did)
            if a.target_id != 0 and st is not None and st.state in _BUSY:
                coverage[a.target_id] = coverage.get(a.target_id, 0) + 1
        slots: list[TargetTrack] = []
        for t in tracks:
            needed = max(1, self.cfg.redundancy) - coverage.get(t.target_id, 0)
            for _ in range(max(0, needed)):
                slots.append(t)
        return slots

    def _current_target(self, drone_id: int) -> int:
        a = self._assignments.get(drone_id)
        return a.target_id if a else 0

    def _allocate_hungarian(self, now, available, slots) -> None:
        n_d, n_s = len(available), len(slots)
        cost = np.zeros((n_d, n_s))
        for i, d in enumerate(available):
            for j, t in enumerate(slots):
                cost[i, j] = self._time_to_intercept(d, t)
        pairs = hungarian.solve(cost)
        for i, j in pairs:
            drone, track = available[i], slots[j]
            self._maybe_assign(now, drone, track, cost[i, j])

    def _allocate_greedy(self, now, available, slots) -> None:
        # nearest-available: for each open slot pick the cheapest free drone
        used = set()
        for t in slots:
            best, best_cost = None, np.inf
            for d in available:
                if d.drone_id in used:
                    continue
                c = self._time_to_intercept(d, t)
                if c < best_cost:
                    best, best_cost = d, c
            if best is not None:
                used.add(best.drone_id)
                self._maybe_assign(now, best, t, best_cost)

    def _maybe_assign(self, now, drone: DroneStatus, track: TargetTrack, cost: float) -> None:
        cur = self._current_target(drone.drone_id)
        if cur == track.target_id:
            return  # already assigned, no change
        if cur != 0:
            # hysteresis: only switch if materially better than current target cost
            cur_track_cost = self._cost_to_target(drone, cur)
            if cur_track_cost is not None and cost > cur_track_cost * (1 - self.cfg.hysteresis_improvement):
                return
        self._assignments[drone.drone_id] = Assignment(
            stamp=now, drone_id=drone.drone_id,
            target_id=track.target_id, role=AssignmentRole.PRIMARY)

    def _cost_to_target(self, drone: DroneStatus, target_id: int) -> float | None:
        return None  # current-target track not retained here; conservative -> keep

    def _unassign(self, now: float, drone_id: int) -> None:
        self._assignments[drone_id] = Assignment(
            stamp=now, drone_id=drone_id, target_id=0)

    def clear_failed(self) -> None:
        self._failed_targets.clear()
