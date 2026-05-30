"""Gateway WebSocket server (PRD viewer §6.2, §7.1).

Single host port (default :8080). Three logical channels multiplexed over one
socket as newline-free JSON text frames, discriminated by ``type``:

* ``frame``  — FrameSnapshot (3D scene + derived grids), state channel.
* ``logs``   — LogBatch (process log), kept off the state channel so a log
  burst can never stall the scene.
* ``control``/``ack``/``status`` — bidirectional control channel
  (recording, replay transport, scenario orchestration).

Live vs. replay share one broadcast path: in live mode the server pumps the
Aggregator/LogBus on a fixed cadence; in replay mode it paces the recorded
``(stamp, message)`` stream. Either way clients receive identical ``frame`` and
``logs`` messages, so the frontend never special-cases replay.
"""

from __future__ import annotations

import asyncio
import json
from typing import Any

import websockets

from .aggregator import Aggregator
from .catalog import list_scenarios
from .config import GatewayConfig
from .logbus import LogBus
from .recorder import Recorder
from .replayer import list_recordings
from .sources import FixtureSource, ReplaySource


class _Clock:
    """Replay transport state: playhead + speed + paused flag."""

    def __init__(self, t0: float, t1: float):
        self.t0 = t0
        self.t1 = t1
        self.playhead = t0
        self.speed = 1.0
        self.paused = False

    def seek(self, t: float):
        self.playhead = max(self.t0, min(self.t1, t))


class GatewayServer:
    def __init__(self, cfg: GatewayConfig, aggregator: Aggregator,
                 logbus: LogBus, recorder: Recorder, source: Any):
        self.cfg = cfg
        self.agg = aggregator
        self.logs = logbus
        self.recorder = recorder
        self.source = source
        self.clients: set = set()
        self._tasks: list[asyncio.Task] = []
        self._clock: _Clock | None = None

    # --- client lifecycle --------------------------------------------------

    async def _handler(self, ws):
        self.clients.add(ws)
        try:
            await ws.send(json.dumps(self._hello()))
            async for raw in ws:
                await self._on_message(ws, raw)
        except websockets.ConnectionClosed:
            pass
        finally:
            self.clients.discard(ws)

    def _hello(self) -> dict:
        return {
            "type": "hello",
            "mode": self.source.mode,
            "source": self.source.name,
            "datum": {"lat": self.cfg.datum.lat, "lon": self.cfg.datum.lon,
                      "alt": self.cfg.datum.alt},
            "snapshotRateHz": self.cfg.snapshot_rate_hz,
            "controlsEnabled": self.cfg.controls_enabled,
            "logRingRows": self.cfg.log_ring_buffer_rows,
        }

    async def _broadcast(self, msg: dict):
        if not self.clients:
            return
        data = json.dumps(msg, separators=(",", ":"))
        dead = []
        for ws in self.clients:
            try:
                await ws.send(data)
            except websockets.ConnectionClosed:
                dead.append(ws)
        for ws in dead:
            self.clients.discard(ws)

    # --- control channel ---------------------------------------------------

    async def _on_message(self, ws, raw: str):
        try:
            msg = json.loads(raw)
        except json.JSONDecodeError:
            await ws.send(json.dumps({"type": "ack", "ok": False,
                                      "error": "malformed json"}))
            return
        if msg.get("type") != "control":
            return
        action = msg.get("action", "")
        try:
            result = await self._dispatch(action, msg)
            await ws.send(json.dumps({"type": "ack", "action": action,
                                      "ok": True, "result": result}))
        except _ControlError as e:
            await ws.send(json.dumps({"type": "ack", "action": action,
                                      "ok": False, "error": str(e)}))

    async def _dispatch(self, action: str, msg: dict) -> Any:
        if action == "recording.status":
            return self.recorder.status
        if action == "recording.start":
            if not self.cfg.recording.enabled:
                raise _ControlError("recording disabled")
            return {"sessionId": self.recorder.start(msg.get("scenario", ""))}
        if action == "recording.stop":
            return {"sessionId": self.recorder.stop()}
        if action == "recordings.list":
            return list_recordings(self.cfg.recording.dir)
        if action == "scenarios.list":
            if not self.cfg.scenarios.enabled:
                raise _ControlError("scenarios disabled")
            return list_scenarios(self.cfg.scenarios.catalog_dir)
        if action in ("scenarios.run", "scenarios.stop", "sim.setSpeed"):
            proxy = getattr(self.source, "scenario_proxy", None)
            if proxy is None:
                raise _ControlError(f"{action} not supported by {self.source.name} source")
            return await proxy(action, msg)
        if action.startswith("replay."):
            return self._replay_control(action, msg)
        raise _ControlError(f"unknown action: {action}")

    def _replay_control(self, action: str, msg: dict) -> Any:
        if self._clock is None:
            raise _ControlError("not in replay mode")
        c = self._clock
        if action == "replay.play":
            c.paused = False
        elif action == "replay.pause":
            c.paused = True
        elif action == "replay.seek":
            c.seek(float(msg.get("stamp", c.t0)))
        elif action == "replay.setSpeed":
            c.speed = max(0.1, min(20.0, float(msg.get("speed", 1.0))))
        else:
            raise _ControlError(f"unknown replay action: {action}")
        return {"playhead": c.playhead, "speed": c.speed, "paused": c.paused,
                "t0": c.t0, "t1": c.t1}

    # --- pumps -------------------------------------------------------------

    async def _live_state_pump(self):
        period = self.cfg.snapshot_period
        while True:
            snap = self.agg.build_snapshot()
            self.recorder.write_state(snap)
            await self._broadcast(snap)
            await asyncio.sleep(period)

    async def _live_log_pump(self):
        period = self.cfg.log_batch_period
        while True:
            batch = self.logs.drain_batch()
            if batch is not None:
                self.recorder.write_logs(batch)
                await self._broadcast(batch)
            await asyncio.sleep(period)

    async def _replay_pump(self):
        src: ReplaySource = self.source
        self._clock = _Clock(src.t0, src.t1)
        c = self._clock
        while True:
            c.playhead = c.t0
            events = src.events()
            prev_stamp = c.t0
            for stamp, message in events:
                while c.paused:
                    await asyncio.sleep(0.05)
                # honor seeks: skip events behind the playhead
                if stamp < c.playhead - 1e-6:
                    prev_stamp = stamp
                    continue
                wait = (stamp - prev_stamp) / max(c.speed, 1e-3)
                if wait > 0:
                    await asyncio.sleep(min(wait, 5.0))
                c.playhead = stamp
                prev_stamp = stamp
                await self._broadcast(message)
            await asyncio.sleep(1.0)  # brief pause, then loop

    # --- run ---------------------------------------------------------------

    async def serve(self):
        if hasattr(self.source, "start"):
            maybe = self.source.start()
            if asyncio.iscoroutine(maybe):
                await maybe

        # let a live source (ros) relay async status onto the control channel
        if hasattr(self.source, "loop"):
            self.source.loop = asyncio.get_running_loop()
            self.source.broadcast = self._broadcast

        if self.source.mode == "replay":
            self._tasks.append(asyncio.create_task(self._replay_pump()))
        else:
            self._tasks.append(asyncio.create_task(self._live_state_pump()))
            self._tasks.append(asyncio.create_task(self._live_log_pump()))

        async with websockets.serve(self._handler, self.cfg.ws_host, self.cfg.ws_port):
            await asyncio.Future()  # run forever

    async def shutdown(self):
        for t in self._tasks:
            t.cancel()
        if hasattr(self.source, "stop"):
            self.source.stop()


class _ControlError(Exception):
    pass
