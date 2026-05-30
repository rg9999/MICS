"""Gateway configuration — mirrors PRD_viewer_architecture.md Appendix A.

Only the server-side keys are modelled here (datum, performance, recording,
scenarios, logging). Layer-visibility defaults are a frontend concern.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import yaml

from .geodesy import Datum

_LEVELS = {"DEBUG": 10, "INFO": 20, "WARN": 30, "ERROR": 40, "FATAL": 50}


@dataclass
class RecordingConfig:
    enabled: bool = True
    dir: str = "./recordings"
    session_name_template: str = "{iso}_run-{seq}"
    segment_rotate_mb: float = 64.0
    segment_rotate_seconds: float = 300.0
    compress: bool = True               # gzip per-segment
    include_logs: bool = True           # logs are always recorded (FR-V-40)


@dataclass
class ScenarioConfig:
    enabled: bool = True
    catalog_dir: str = "./scenarios"
    orchestrator_action: str = "run_scenario"
    status_topic: str = "/sim_run_status"
    set_speed_service: str = "set_sim_speed"
    auto_record_on_run: bool = True
    allow_overrides: bool = True
    default_rtf: float = 2.0
    allow_runtime_speed: bool = False   # auto-disabled in the software-sim profile


@dataclass
class GatewayConfig:
    ws_host: str = "0.0.0.0"
    ws_port: int = 8080
    datum: Datum = field(default_factory=lambda: Datum(lat=32.0853, lon=34.7818, alt=30.0))

    snapshot_rate_hz: float = 25.0
    lidar_max_points: int = 50000

    # process-log channel (separate from the snapshot channel)
    log_topic: str = "/rosout"
    log_min_level: int = 20             # INFO; gateway drops below this
    log_ring_buffer_rows: int = 20000   # advisory, surfaced to clients
    log_batch_rate_hz: float = 10.0

    controls_enabled: bool = False      # read-only unless explicitly enabled + authed

    recording: RecordingConfig = field(default_factory=RecordingConfig)
    scenarios: ScenarioConfig = field(default_factory=ScenarioConfig)

    @property
    def snapshot_period(self) -> float:
        return 1.0 / self.snapshot_rate_hz if self.snapshot_rate_hz > 0 else 0.04

    @property
    def log_batch_period(self) -> float:
        return 1.0 / self.log_batch_rate_hz if self.log_batch_rate_hz > 0 else 0.1


def _level_to_int(v) -> int:
    if isinstance(v, int):
        return v
    return _LEVELS.get(str(v).upper(), 20)


def load_config(path: str | Path | None) -> GatewayConfig:
    cfg = GatewayConfig()
    if path is None:
        return cfg
    data = yaml.safe_load(Path(path).read_text()) or {}

    conn = data.get("connection", {}) or {}
    cfg.ws_port = int(conn.get("ws_port", _port_from_url(conn.get("ws_url"), cfg.ws_port)))
    cfg.ws_host = str(conn.get("ws_host", cfg.ws_host))

    d = data.get("datum", {}) or {}
    cfg.datum = Datum(
        lat=float(d.get("lat", cfg.datum.lat)),
        lon=float(d.get("lon", cfg.datum.lon)),
        alt=float(d.get("alt", cfg.datum.alt)),
    )

    perf = data.get("performance", {}) or {}
    cfg.snapshot_rate_hz = float(perf.get("snapshot_rate_hz", cfg.snapshot_rate_hz))
    cfg.lidar_max_points = int(perf.get("lidar_max_points", cfg.lidar_max_points))

    grids = data.get("grids", {}) or {}
    plog = grids.get("process_log", {}) or {}
    cfg.log_topic = str(plog.get("source_topic", cfg.log_topic))
    cfg.log_min_level = _level_to_int(plog.get("min_level", cfg.log_min_level))
    cfg.log_ring_buffer_rows = int(plog.get("ring_buffer_rows", cfg.log_ring_buffer_rows))
    cfg.log_batch_rate_hz = float(plog.get("batch_rate_hz", cfg.log_batch_rate_hz))

    ctl = data.get("controls", {}) or {}
    cfg.controls_enabled = bool(ctl.get("enabled", cfg.controls_enabled))

    rec = data.get("recording", {}) or {}
    cfg.recording = RecordingConfig(
        enabled=bool(rec.get("enabled", True)),
        dir=str(rec.get("dir", "./recordings")),
        session_name_template=str(rec.get("session_name_template", "{iso}_run-{seq}")),
        segment_rotate_mb=float(rec.get("segment_rotate_mb", 64.0)),
        segment_rotate_seconds=float(rec.get("segment_rotate_seconds", 300.0)),
        compress=str(rec.get("compress", "gzip")).lower() != "none",
        include_logs=bool(rec.get("include_logs", True)),
    )

    sc = data.get("scenarios", {}) or {}
    cfg.scenarios = ScenarioConfig(
        enabled=bool(sc.get("enabled", True)),
        catalog_dir=str(sc.get("catalog_dir", "./scenarios")),
        orchestrator_action=str(sc.get("orchestrator_action", "run_scenario")),
        status_topic=str(sc.get("status_topic", "/sim_run_status")),
        set_speed_service=str(sc.get("set_speed_service", "set_sim_speed")),
        auto_record_on_run=bool(sc.get("auto_record_on_run", True)),
        allow_overrides=bool(sc.get("allow_overrides", True)),
        default_rtf=float(sc.get("default_rtf", 2.0)),
        allow_runtime_speed=bool(sc.get("allow_runtime_speed", False)),
    )
    return cfg


def _port_from_url(url, default: int) -> int:
    if not url:
        return default
    try:
        return int(str(url).rsplit(":", 1)[1].split("/")[0])
    except (IndexError, ValueError):
        return default
