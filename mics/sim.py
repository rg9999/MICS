"""Simulation orchestrator — ties the whole MICS core together (PRD §5, §9).

The world loop, each tick:
  1. step attackers (ground truth)
  2. cue_degrader samples truth -> partial GS tracks (internal source)
  3. track_manager ages/fuses -> /tracks
  4. allocator computes /assignments from roster + tracks
  5. each drone steps its state machine (sensors -> fusion -> guidance -> safety)
  6. roster updated from drone status; metrics collected

Produces a Metrics summary (PRD §9.3): capture success rate, mean time-to-
intercept, reassignment latency, min inter-drone separation, false-fire count.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .allocator import Allocator
from .attacker import Attacker
from .config import ScenarioConfig
from .cue_degrader import CueDegrader
from .drone import Drone
from .geometry import norm
from .msgs import CaptureResult, DroneState
from .sensors import SensorSuite
from .track_manager import TrackManager


@dataclass
class Metrics:
    captures: int = 0
    misses: int = 0
    n_targets: int = 0
    time_to_intercept: list = field(default_factory=list)
    reassignments: int = 0
    min_separation_m: float = float("inf")
    false_fires: int = 0
    handoffs: int = 0
    sim_time_s: float = 0.0

    def summary(self) -> dict:
        n = max(self.n_targets, 1)
        mtti = float(np.mean(self.time_to_intercept)) if self.time_to_intercept else None
        return {
            "capture_success_rate": self.captures / n,
            "captures": self.captures,
            "misses": self.misses,
            "n_targets": self.n_targets,
            "mean_time_to_intercept_s": mtti,
            "reassignments": self.reassignments,
            "min_inter_drone_separation_m": (None if self.min_separation_m == float("inf")
                                             else round(self.min_separation_m, 2)),
            "false_fires": self.false_fires,
            "handoffs": self.handoffs,
            "sim_time_s": round(self.sim_time_s, 2),
        }


class Simulation:
    def __init__(self, cfg: ScenarioConfig):
        self.cfg = cfg
        self.rng = np.random.default_rng(cfg.seed)
        self.t = 0.0
        self.metrics = Metrics()
        self._handoff_seen: set = set()
        self._prev_assignment_targets: dict = {}
        self._reassign_pending: dict = {}  # drone_id -> time failure observed

        # attackers
        self.attackers: list[Attacker] = []
        for ac in cfg.attackers:
            self.attackers.append(Attacker(
                target_id=ac.id, profile=ac.profile, speed_mps=ac.speed_mps,
                position=np.array(ac.start, dtype=float),
                asset=np.array(ac.asset, dtype=float),
                waypoints=[np.array(w, dtype=float) for w in ac.waypoints],
                r_evade_m=ac.r_evade_m,
                rng=np.random.default_rng(cfg.seed + 100 + ac.id),
            ))
        self.metrics.n_targets = len(self.attackers)

        # defenders, spread along a line near origin
        self.drones: list[Drone] = []
        n = cfg.n_defenders
        for i in range(n):
            x = (i - (n - 1) / 2) * cfg.defender_spacing_m
            d = Drone(
                drone_id=i + 1,
                position=np.array([x, -50.0, 50.0]),
                params=cfg.drone,
                safety_cfg=cfg.safety,
                rng=np.random.default_rng(cfg.seed + 200 + i),
            )
            d.sensors = SensorSuite(
                mode=cfg.sensor_mode, rng=d.rng,
                camera_enabled=cfg.camera_enabled,
                radar_enabled=cfg.radar_enabled,
                lidar_enabled=cfg.lidar_enabled,
            )
            self.drones.append(d)

        # ground station
        self.track_manager = TrackManager()
        self.allocator = Allocator(cfg.allocation)
        self.cue_degrader = CueDegrader(
            pos_sigma_m=cfg.cue_pos_sigma_m,
            dropout_pct=cfg.cue_dropout_pct,
            latency_ms=cfg.cue_latency_ms,
            rng=np.random.default_rng(cfg.seed + 1),
        )
        self._tti_start: dict = {}  # target_id -> first-assigned time

    # --- main loop --------------------------------------------------------

    def run(self) -> Metrics:
        steps = int(self.cfg.max_time_s / self.cfg.dt)
        for _ in range(steps):
            self.step()
            if self._all_done():
                break
        self.metrics.sim_time_s = self.t
        return self.metrics

    def _all_done(self) -> bool:
        return all(not a.alive for a in self.attackers)

    def _attacker(self, target_id: int):
        for a in self.attackers:
            if a.target_id == target_id:
                return a
        return None

    def step(self) -> None:
        dt = self.cfg.dt
        self.t += dt
        now = self.t

        defender_pos = [d.position for d in self.drones]

        # 1. attackers
        for a in self.attackers:
            a.step(dt, defender_pos)

        # 2. cue degradation (internal source)
        truth = [a.state() for a in self.attackers]
        self.cue_degrader.ingest(now, truth)
        released = self.cue_degrader.release(now)

        # 3. track manager
        self.track_manager.ingest(now, released)
        self.track_manager.step(now)
        tracks = self.track_manager.tracks()

        # 4. allocation
        statuses = [d.status(now) for d in self.drones]
        self.allocator.update_roster(statuses)
        assignments = self.allocator.allocate(now, tracks)
        self._track_reassignments(now, assignments)

        # push assignments to drones
        for d in self.drones:
            a = assignments.get(d.drone_id)
            if a is not None:
                d.set_assignment(a)

        # 5. drones step
        truth_by_id = {a.target_id: a.state() for a in self.attackers}
        for d in self.drones:
            cue = None
            atruth = None
            if d.assignment and d.assignment.target_id != 0:
                cue = self.track_manager.get(d.assignment.target_id)
                atruth = truth_by_id.get(d.assignment.target_id)
                if d.assignment.target_id not in self._tti_start:
                    self._tti_start[d.assignment.target_id] = now
            d.step(now, dt, cue, atruth, statuses)

        # 6. metrics: handoffs, captures, separation, false-fires
        self._collect_metrics(now)

    # --- metrics helpers --------------------------------------------------

    def _track_reassignments(self, now, assignments) -> None:
        for did, a in assignments.items():
            prev = self._prev_assignment_targets.get(did, 0)
            if prev != a.target_id and a.target_id != 0 and prev != 0:
                self.metrics.reassignments += 1
            self._prev_assignment_targets[did] = a.target_id

    def _collect_metrics(self, now) -> None:
        # min inter-drone separation among airborne drones
        air = [d for d in self.drones if d.state in
               (DroneState.MIDCOURSE, DroneState.ACQUIRING, DroneState.TERMINAL)]
        for i in range(len(air)):
            for j in range(i + 1, len(air)):
                sep = norm(air[i].position - air[j].position)
                self.metrics.min_separation_m = min(self.metrics.min_separation_m, sep)

        for d in self.drones:
            if d.state == DroneState.TERMINAL and d.drone_id not in self._handoff_seen:
                self._handoff_seen.add(d.drone_id)
                self.metrics.handoffs += 1

            # drain capture events. Truth decides the outcome: a declared
            # SUCCESS only counts as a real capture if the true target is
            # actually within capture range of the engagement point; otherwise
            # it is a FALSE FIRE (PRD §9.3: must be 0).
            for ev in d.capture_events:
                if ev.result == CaptureResult.SUCCESS:
                    atk = self._attacker(ev.target_id)
                    r_cap = self.cfg.drone.r_capture_m
                    if (atk is not None and atk.alive
                            and norm(atk.position - ev.engagement_point) <= r_cap + 0.5):
                        self.metrics.captures += 1
                        if ev.target_id in self._tti_start:
                            self.metrics.time_to_intercept.append(
                                now - self._tti_start[ev.target_id])
                        atk.kill()
                    else:
                        self.metrics.false_fires += 1
                elif ev.result == CaptureResult.MISS:
                    self.metrics.misses += 1
            d.capture_events.clear()
