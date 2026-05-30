"""Bus-state aggregator + per-frame snapshot construction (PRD viewer §6.2, §8).

Holds the latest value of every bus entity (drones, GS tracks, assignments,
onboard estimates) and a buffer of capture events. On each render tick it
coalesces them into one ``FrameSnapshot`` dict, transforming ENU->geodetic once
and computing the grid-derived fields (range-to-target, allocation status,
speed, altitude, ETA) so clients render rather than recompute.

Input dataclasses are ROS-free: ``ros_source`` and ``fixture`` both build them.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from threading import Lock

import numpy as np

from .geodesy import Transformer

STATE_NAMES = ["IDLE", "ASSIGNED", "MIDCOURSE", "ACQUIRING",
               "TERMINAL", "CAPTURED", "FAILED"]
RESULT_NAMES = ["attempt", "success", "miss"]
SOURCE_NAMES = ["internal_sim", "external", "onboard"]
ROLE_NAMES = ["primary", "standby"]

# states in which a drone is actively engaging (used for allocation status)
_ACTIVE_STATES = {"ASSIGNED", "MIDCOURSE", "ACQUIRING", "TERMINAL"}


def _name(table: list[str], idx: int, default: str = "unknown") -> str:
    return table[idx] if 0 <= idx < len(table) else default


@dataclass
class GwDrone:
    drone_id: int
    enu: np.ndarray
    vel_enu: np.ndarray
    state: int = 0
    current_target: int = 0
    battery_pct: float = 100.0
    track_quality: float = 0.0
    stamp: float = 0.0


@dataclass
class GwTrack:
    target_id: int
    enu: np.ndarray
    vel_enu: np.ndarray | None = None
    cov_enu: np.ndarray = field(default_factory=lambda: np.eye(3))
    class_confidence: float = 0.0
    source: int = 0
    age: float = 0.0
    stamp: float = 0.0


@dataclass
class GwAssignment:
    drone_id: int
    target_id: int = 0
    role: int = 0


@dataclass
class GwCapture:
    drone_id: int
    target_id: int
    result: int
    enu: np.ndarray
    stamp: float = 0.0


@dataclass
class GwLog:
    stamp: float
    level: int
    source: str
    msg: str
    file: str = ""
    func: str = ""
    line: int = 0


class Aggregator:
    def __init__(self, transformer: Transformer):
        self.tf = transformer
        self._lock = Lock()
        self._drones: dict[int, GwDrone] = {}
        self._tracks: dict[int, GwTrack] = {}
        self._assignments: dict[int, GwAssignment] = {}
        self._estimates: dict[int, GwTrack] = {}    # by drone_id (onboard fused)
        self._events: list[GwCapture] = []          # drained each frame
        self._captured: set[int] = set()            # persistent terminal state
        self._sim_t = 0.0

    # --- ingest (called from a source thread) ------------------------------

    def set_time(self, t: float):
        with self._lock:
            self._sim_t = float(t)

    def ingest_drone(self, d: GwDrone):
        with self._lock:
            self._drones[d.drone_id] = d

    def ingest_track(self, t: GwTrack):
        with self._lock:
            self._tracks[t.target_id] = t

    def ingest_assignment(self, a: GwAssignment):
        with self._lock:
            self._assignments[a.drone_id] = a

    def ingest_estimate(self, drone_id: int, t: GwTrack):
        with self._lock:
            self._estimates[drone_id] = t

    def ingest_capture(self, c: GwCapture):
        with self._lock:
            self._events.append(c)
            if c.result == 1:  # success
                self._captured.add(c.target_id)

    def expire_track(self, target_id: int):
        with self._lock:
            self._tracks.pop(target_id, None)

    def clear(self):
        with self._lock:
            self._drones.clear()
            self._tracks.clear()
            self._assignments.clear()
            self._estimates.clear()
            self._events.clear()
            self._captured.clear()
            self._sim_t = 0.0

    # --- snapshot ----------------------------------------------------------

    def _track_view(self, t: GwTrack) -> dict:
        v = {
            "targetId": t.target_id,
            "position": self.tf.enu_to_lonlatalt(t.enu),
            "enu": [float(x) for x in t.enu],
            "source": _name(SOURCE_NAMES, t.source),
            "classConfidence": float(t.class_confidence),
            "age": float(t.age),
            "posSigmaM": float(math.sqrt(max(np.trace(t.cov_enu), 0.0) / 3.0)),
            "covEnu": [float(x) for x in np.asarray(t.cov_enu).reshape(9)],
        }
        v["velEnu"] = ([float(x) for x in t.vel_enu]
                       if t.vel_enu is not None else None)
        return v

    def build_snapshot(self) -> dict:
        with self._lock:
            drones = list(self._drones.values())
            tracks = dict(self._tracks)
            assignments = dict(self._assignments)
            estimates = dict(self._estimates)
            events = self._events
            self._events = []
            captured = set(self._captured)
            sim_t = self._sim_t

        # primary assignment target per drone (target_id 0 == unassigned)
        alloc_by_drone = {a.drone_id: a.target_id for a in assignments.values()
                          if a.target_id != 0 and a.role == 0}
        # which drone is engaging each target
        engaging = {}
        for a in assignments.values():
            if a.target_id != 0 and a.role == 0:
                d = self._drones.get(a.drone_id)
                if d is not None and _name(STATE_NAMES, d.state) in _ACTIVE_STATES:
                    engaging.setdefault(a.target_id, a.drone_id)

        drones_out, drones_derived = [], []
        for d in drones:
            tid = alloc_by_drone.get(d.drone_id)
            tgt = tracks.get(tid) if tid else None
            rng = eta = None
            if tgt is not None:
                rel = tgt.enu - d.enu
                rng = float(np.linalg.norm(rel))
                if rng > 1e-6:
                    los = rel / rng
                    v_tgt = tgt.vel_enu if tgt.vel_enu is not None else np.zeros(3)
                    closing = float(np.dot(d.vel_enu - np.asarray(v_tgt), los))
                    if closing > 0.1:
                        eta = rng / closing
            speed = float(np.linalg.norm(d.vel_enu))
            altitude = float(d.enu[2])
            drones_out.append({
                "droneId": d.drone_id,
                "state": _name(STATE_NAMES, d.state),
                "currentTarget": d.current_target,
                "batteryPct": float(d.battery_pct),
                "trackQuality": float(d.track_quality),
                "position": self.tf.enu_to_lonlatalt(d.enu),
                "enu": [float(x) for x in d.enu],
                "velEnu": [float(x) for x in d.vel_enu],
            })
            drones_derived.append({
                "droneId": d.drone_id,
                "speed": speed,
                "altitude": altitude,
                "allocatedTarget": (tid if tid else None),
                "rangeToTarget": rng,
                "etaToInterceptS": eta,
            })

        tracks_out, targets_derived = [], []
        for t in tracks.values():
            tracks_out.append(self._track_view(t))
            if t.target_id in captured:
                alloc = {"kind": "CAPTURED"}
            elif t.target_id in engaging:
                alloc = {"kind": "ENGAGED", "byDrone": engaging[t.target_id]}
            else:
                alloc = {"kind": "UNENGAGED"}
            speed = (float(np.linalg.norm(t.vel_enu))
                     if t.vel_enu is not None else None)
            targets_derived.append({
                "targetId": t.target_id,
                "speed": speed,
                "altitude": float(t.enu[2]),
                "allocation": alloc,
            })

        return {
            "type": "frame",
            "stamp": sim_t,
            "datum": {"lat": self.tf.datum.lat, "lon": self.tf.datum.lon,
                      "alt": self.tf.datum.alt},
            "drones": drones_out,
            "tracks": tracks_out,
            "assignments": [
                {"droneId": a.drone_id, "targetId": a.target_id,
                 "role": _name(ROLE_NAMES, a.role)}
                for a in assignments.values()
            ],
            "estimates": {str(did): self._track_view(t)
                          for did, t in estimates.items()},
            "events": [
                {"stamp": e.stamp, "droneId": e.drone_id, "targetId": e.target_id,
                 "result": _name(RESULT_NAMES, e.result),
                 "position": self.tf.enu_to_lonlatalt(e.enu),
                 "enu": [float(x) for x in e.enu]}
                for e in events
            ],
            "dronesDerived": drones_derived,
            "targetsDerived": targets_derived,
        }
