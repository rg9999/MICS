"""Scenario catalog reader (PRD viewer §6.2 #8, FR-V-44).

The gateway lists the vetted scenario YAMLs directly (no ROS round-trip needed
to enumerate). The server-side ``path`` is NEVER forwarded to the browser; the
client only ever references a scenario by its catalog ``scenarioId``.
"""

from __future__ import annotations

from pathlib import Path

import yaml


def _scenario_info(path: Path) -> dict:
    try:
        data = yaml.safe_load(path.read_text()) or {}
    except yaml.YAMLError:
        data = {}
    defenders = data.get("defenders", {}) or {}
    attackers = data.get("attackers", []) or []
    return {
        "scenarioId": path.stem,
        "name": str(data.get("name", path.stem.replace("_", " ").title())),
        "description": str(data.get("description", "")),
        "targetSource": str(data.get("target_source", "internal")),
        "defenderCount": int(defenders.get("count", 0)),
        "attackerCount": len(attackers),
        "sensorMode": str(data.get("sensor_mode", "ideal")),
        # server-side path intentionally omitted from the client-facing dict
    }


def list_scenarios(catalog_dir: str | Path) -> list[dict]:
    d = Path(catalog_dir)
    if not d.is_dir():
        return []
    out = []
    for p in sorted(d.glob("*.y*ml")):
        out.append(_scenario_info(p))
    return out
