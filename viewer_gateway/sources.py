"""State sources feeding the gateway (PRD viewer §6.2, §6.5).

Two of the three sources live here because they are ROS-free and therefore
unit-testable on any host:

* ``FixtureSource`` — synthetic drone/target motion, drives the Aggregator +
  LogBus directly. Used for frontend/dev work and for smoke-testing the server
  without a running sim.
* ``ReplaySource`` — re-emits a recorded session's byte-identical messages
  (see :mod:`viewer_gateway.replayer`). It is paced by the server clock.

The live ROS source lives in :mod:`viewer_gateway.ros_source` so importing this
module never pulls in rclpy.

Source modes:
* ``"live"``  — the source mutates the shared Aggregator/LogBus; the server
  pumps snapshots/log-batches off them on its own cadence.
* ``"replay"`` — the source yields ``(stamp, message)`` pairs that the server
  paces and forwards verbatim.
"""

from __future__ import annotations

import math
import threading
import time
from typing import Iterator

import numpy as np

from .aggregator import (Aggregator, GwAssignment, GwCapture, GwDrone, GwLog,
                         GwTrack)
from .logbus import LogBus
from .replayer import iter_merged, load_manifest


class FixtureSource:
    """Synthetic scene: defenders orbit/intercept a handful of inbound targets.

    Deterministic, ROS-free. Runs a background thread that advances a software
    clock and pushes state into the Aggregator + LogBus, mimicking what the
    live ROS source would do.
    """

    mode = "live"
    name = "fixture"

    def __init__(self, aggregator: Aggregator, logbus: LogBus,
                 n_drones: int = 3, n_targets: int = 2, dt: float = 0.04):
        self.agg = aggregator
        self.logs = logbus
        self.n_drones = n_drones
        self.n_targets = n_targets
        self.dt = dt
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._t = 0.0
        self._captured: set[int] = set()

    def start(self):
        if self._thread is not None:
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="fixture", daemon=True)
        self._thread.start()
        self.logs.ingest(GwLog(stamp=0.0, level=20, source="fixture",
                               msg="fixture source started", func="start"))

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None

    def reset(self):
        self.agg.clear()
        self._t = 0.0
        self._captured.clear()

    # --- synthetic scene ---------------------------------------------------

    def _target_pos(self, i: int, t: float) -> np.ndarray:
        # inbound from ~3 km out, descending, slight cross-track weave
        x0 = 3000.0 - 60.0 * t
        y0 = 400.0 * (i - (self.n_targets - 1) / 2.0) + 80.0 * math.sin(0.2 * t)
        z0 = max(40.0, 350.0 - 6.0 * t)
        return np.array([x0, y0, z0])

    def _step(self):
        t = self._t
        # targets
        tgt_pos: dict[int, np.ndarray] = {}
        for i in range(self.n_targets):
            tid = 100 + i
            if tid in self._captured:
                continue
            p = self._target_pos(i, t)
            v = (p - self._target_pos(i, t - self.dt)) / self.dt
            tgt_pos[tid] = p
            self.agg.ingest_track(GwTrack(
                target_id=tid, enu=p, vel_enu=v,
                cov_enu=np.eye(3) * (25.0 + 5.0 * math.sin(0.5 * t + i)),
                class_confidence=0.7 + 0.2 * math.sin(0.3 * t),
                source=0, age=0.0, stamp=t))

        # drones: each chases the nearest live target
        for j in range(self.n_drones):
            did = j + 1
            home = np.array([200.0 * (j - (self.n_drones - 1) / 2.0), -200.0, 60.0])
            tid, tp = self._nearest_target(home, tgt_pos)
            if tp is not None:
                los = tp - home
                dist = float(np.linalg.norm(los))
                frac = min(1.0, (t % 60.0) / 40.0)
                pos = home + los * frac
                vel = los / max(dist, 1.0) * 45.0
                state = 2 if frac < 0.6 else (3 if frac < 0.9 else 4)
                if frac >= 0.98 and tid not in self._captured:
                    self._captured.add(tid)
                    self.agg.ingest_capture(GwCapture(
                        drone_id=did, target_id=tid, result=1,
                        enu=tp, stamp=t))
                    self.agg.expire_track(tid)
                    self.logs.ingest(GwLog(stamp=t, level=20, source=f"drone_{did}",
                                           msg=f"target {tid} captured", func="capture"))
                    state = 5
                self.agg.ingest_assignment(GwAssignment(
                    drone_id=did, target_id=tid, role=0))
            else:
                pos = home
                vel = np.zeros(3)
                state = 0
                self.agg.ingest_assignment(GwAssignment(drone_id=did, target_id=0, role=0))
            self.agg.ingest_drone(GwDrone(
                drone_id=did, enu=pos, vel_enu=vel, state=state,
                current_target=tid if tp is not None else 0,
                battery_pct=max(20.0, 100.0 - 0.3 * t),
                track_quality=0.6 + 0.3 * math.sin(0.4 * t + j), stamp=t))

        self.agg.set_time(t)

    @staticmethod
    def _nearest_target(origin, tgt_pos):
        best_id, best_p, best_d = None, None, float("inf")
        for tid, p in tgt_pos.items():
            d = float(np.linalg.norm(p - origin))
            if d < best_d:
                best_id, best_p, best_d = tid, p, d
        return best_id, best_p

    def _run(self):
        next_t = time.monotonic()
        ticks = 0
        while not self._stop.is_set():
            self._step()
            self._t += self.dt
            ticks += 1
            if ticks % 250 == 0:
                self.logs.ingest(GwLog(stamp=self._t, level=20, source="fixture",
                                       msg=f"sim_t={self._t:.1f}s", func="_run"))
            next_t += self.dt
            sleep = next_t - time.monotonic()
            if sleep > 0:
                self._stop.wait(sleep)
            else:
                next_t = time.monotonic()
            # loop the scenario so the fixture never goes idle forever
            if len(self._captured) >= self.n_targets and self._t > 2.0:
                self._stop.wait(2.0)
                self.reset()


class ReplaySource:
    """Re-emits a recorded session. Yields pre-built ``(stamp, message)`` pairs.

    The server owns pacing (play/pause/speed/seek); this source just provides
    the ordered event stream and the manifest for the timeline.
    """

    mode = "replay"
    name = "replay"

    def __init__(self, session_dir: str):
        self.session_dir = session_dir
        self.manifest = load_manifest(session_dir)

    def events(self) -> Iterator[tuple[float, dict]]:
        return iter_merged(self.session_dir)

    @property
    def t0(self) -> float:
        return self._span()[0]

    @property
    def t1(self) -> float:
        return self._span()[1]

    def _span(self) -> tuple[float, float]:
        segs = (self.manifest.get("stateSegments", [])
                + self.manifest.get("logSegments", []))
        if not segs:
            return (0.0, 0.0)
        lo = min(s.get("startStamp", 0.0) for s in segs)
        hi = max(s.get("endStamp", 0.0) for s in segs)
        return (float(lo), float(hi))
