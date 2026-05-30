"""Scenario config loader — PRD Appendix A.

Loads the illustrative scenario YAML into typed config objects. Supports the
CPU-laptop overlay (Appendix A.1) keys (render/headless, sensor enables,
point_decimation, detector) where they affect the pure-Python core.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import yaml

from .allocator import AllocatorConfig
from .drone import DroneParams
from .safety import SafetyConfig


@dataclass
class AttackerCfg:
    id: int
    profile: str = "ingress"
    speed_mps: float = 15.0
    r_evade_m: float = 80.0
    start: list = field(default_factory=lambda: [-300.0, 0.0, 50.0])
    asset: list = field(default_factory=lambda: [300.0, 0.0, 50.0])
    waypoints: list = field(default_factory=list)


@dataclass
class ScenarioConfig:
    target_source: str = "internal"
    sensor_mode: str = "ideal"
    render: str = "headless"
    n_defenders: int = 4
    autopilot: str = "px4"
    airframe: str = "holybro_x500v2"
    defender_spacing_m: float = 20.0
    attackers: list = field(default_factory=list)

    cue_pos_sigma_m: float = 15.0
    cue_dropout_pct: float = 20.0
    cue_latency_ms: float = 300.0

    allocation: AllocatorConfig = field(default_factory=AllocatorConfig)
    drone: DroneParams = field(default_factory=DroneParams)
    safety: SafetyConfig = field(default_factory=SafetyConfig)

    camera_enabled: bool = True
    radar_enabled: bool = True
    lidar_enabled: bool = True

    seed: int = 0
    dt: float = 0.05
    max_time_s: float = 120.0


def load_scenario(path: str | Path) -> ScenarioConfig:
    data = yaml.safe_load(Path(path).read_text())
    return from_dict(data)


def from_dict(data: dict) -> ScenarioConfig:
    cfg = ScenarioConfig()
    cfg.target_source = data.get("target_source", cfg.target_source)
    cfg.sensor_mode = data.get("sensor_mode", cfg.sensor_mode)
    cfg.render = data.get("render", cfg.render)
    cfg.seed = int(data.get("seed", cfg.seed))
    cfg.dt = float(data.get("sim", {}).get("dt", cfg.dt)) if isinstance(data.get("sim"), dict) else cfg.dt
    cfg.max_time_s = float(data.get("max_time_s", cfg.max_time_s))

    d = data.get("defenders", {}) or {}
    cfg.n_defenders = int(d.get("count", cfg.n_defenders))
    cfg.autopilot = d.get("autopilot", cfg.autopilot)
    cfg.airframe = d.get("airframe", cfg.airframe)

    for a in data.get("attackers", []) or []:
        cfg.attackers.append(AttackerCfg(
            id=int(a.get("id", 1)),
            profile=a.get("profile", "ingress"),
            speed_mps=float(a.get("speed_mps", 15.0)),
            r_evade_m=float(a.get("r_evade_m", 80.0)),
            start=a.get("start", [-300.0, 0.0, 50.0]),
            asset=a.get("asset", [300.0, 0.0, 50.0]),
            waypoints=a.get("waypoints", []),
        ))
    if not cfg.attackers:
        cfg.attackers.append(AttackerCfg(id=1))

    cd = data.get("cue_degradation", {}) or {}
    cfg.cue_pos_sigma_m = float(cd.get("pos_sigma_m", cfg.cue_pos_sigma_m))
    cfg.cue_dropout_pct = float(cd.get("dropout_pct", cfg.cue_dropout_pct))
    cfg.cue_latency_ms = float(cd.get("latency_ms", cfg.cue_latency_ms))

    al = data.get("allocation", {}) or {}
    cfg.allocation = AllocatorConfig(
        algorithm=al.get("algorithm", "hungarian"),
        redundancy=int(al.get("redundancy", 1)),
        reassign_max_latency_s=float(al.get("reassign_max_latency_s", 1.0)),
    )

    tm = data.get("terminal", {}) or {}
    cfg.drone = DroneParams(
        nav_constant=float(tm.get("nav_constant", 3.0)),
        r_capture_m=float(tm.get("r_capture_m", 3.0)),
    )

    sf = data.get("safety", {}) or {}
    cfg.safety = SafetyConfig(
        geofence_radius_m=float(sf.get("geofence_radius_m", 500.0)),
        capture_min_teammate_sep_m=float(sf.get("capture_min_teammate_sep_m", 10.0)),
    )

    sensors = data.get("sensors", {}) or {}
    cfg.camera_enabled = bool((sensors.get("camera", {}) or {}).get("enabled", True))
    cfg.radar_enabled = bool((sensors.get("radar", {}) or {}).get("enabled", True))
    cfg.lidar_enabled = bool((sensors.get("lidar", {}) or {}).get("enabled", True))
    return cfg
