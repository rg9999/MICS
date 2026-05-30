"""Live ROS source (PRD viewer §6.2 #1-6, §6.4).

The ONLY gateway module that imports rclpy. It subscribes to the MICS bus,
maps each message into the ROS-free ``Gw*`` dataclasses, and feeds them into the
shared Aggregator + LogBus — exactly the contract the fixture source mimics, so
the server's pumps are source-agnostic.

It also hosts the scenario-orchestration proxy: a RunScenario action client + a
SetSimSpeed service client against ``mics_sim_orchestrator``, plus a relay of
``/sim_run_status`` out to clients. rclpy spins on a background thread; the proxy
bridges to the asyncio control channel via ``run_coroutine_threadsafe``.
"""

from __future__ import annotations

import threading

from .aggregator import (Aggregator, GwAssignment, GwCapture, GwDrone, GwLog,
                         GwTrack)
from .config import GatewayConfig
from .logbus import LogBus


def _stamp(header) -> float:
    return float(header.stamp.sec) + float(header.stamp.nanosec) * 1e-9


class RosSource:
    mode = "live"
    name = "ros"

    def __init__(self, cfg: GatewayConfig, aggregator: Aggregator, logbus: LogBus):
        self.cfg = cfg
        self.agg = aggregator
        self.logs = logbus
        self._node = None
        self._executor = None
        self._thread: threading.Thread | None = None
        self._action_client = None
        self._speed_client = None

    # --- lifecycle ---------------------------------------------------------

    def start(self):
        import numpy as np
        import rclpy
        from rclpy.node import Node
        from rclpy.executors import MultiThreadedExecutor

        from rcl_interfaces.msg import Log
        from std_msgs.msg import Float64, Int32
        from mics_msgs.msg import (Assignment, CaptureEvent, DroneStatus,
                                   TargetTrack, SimRunStatus)
        from mics_msgs.action import RunScenario
        from mics_msgs.srv import SetSimSpeed
        from rclpy.action import ActionClient

        if not rclpy.ok():
            rclpy.init()
        node = Node("viewer_gateway_source")
        self._node = node
        self._np = np

        node.create_subscription(DroneStatus, "/state_sharing", self._on_drone, 50)
        node.create_subscription(TargetTrack, "/tracks", self._on_track, 50)
        node.create_subscription(Assignment, "/assignments", self._on_assign, 50)
        node.create_subscription(CaptureEvent, "/capture_events", self._on_capture, 50)
        node.create_subscription(Float64, "/sim/clock", self._on_clock, 50)
        node.create_subscription(Log, self.cfg.log_topic, self._on_log, 200)

        sc = self.cfg.scenarios
        self._action_client = ActionClient(node, RunScenario, sc.orchestrator_action)
        self._speed_client = node.create_client(SetSimSpeed, sc.set_speed_service)
        node.create_subscription(SimRunStatus, sc.status_topic, self._on_run_status, 10)
        self._RunScenario = RunScenario
        self._SetSimSpeed = SetSimSpeed

        self._executor = MultiThreadedExecutor()
        self._executor.add_node(node)
        self._thread = threading.Thread(target=self._executor.spin,
                                        name="rclpy-spin", daemon=True)
        self._thread.start()

        # set by serve() so the proxy can push status frames to clients
        self.loop = None
        self.broadcast = None

    def stop(self):
        if self._executor is not None:
            self._executor.shutdown()
        if self._node is not None:
            self._node.destroy_node()

    # --- bus -> aggregator -------------------------------------------------

    def _on_drone(self, m):
        np = self._np
        p = m.pose.position
        v = m.twist.linear
        self.agg.ingest_drone(GwDrone(
            drone_id=int(m.drone_id),
            enu=np.array([p.x, p.y, p.z]),
            vel_enu=np.array([v.x, v.y, v.z]),
            state=int(m.state), current_target=int(m.current_target),
            battery_pct=float(m.battery_pct), track_quality=float(m.track_quality),
            stamp=_stamp(m.header)))

    def _on_track(self, m):
        np = self._np
        p = m.position
        vel = np.array([m.velocity.x, m.velocity.y, m.velocity.z]) if m.has_velocity else None
        self.agg.ingest_track(GwTrack(
            target_id=int(m.target_id),
            enu=np.array([p.x, p.y, p.z]), vel_enu=vel,
            cov_enu=np.array(m.position_covariance, dtype=float).reshape(3, 3),
            class_confidence=float(m.class_confidence), source=int(m.source),
            age=float(m.age), stamp=_stamp(m.header)))

    def _on_assign(self, m):
        self.agg.ingest_assignment(GwAssignment(
            drone_id=int(m.drone_id), target_id=int(m.target_id), role=int(m.role)))

    def _on_capture(self, m):
        np = self._np
        p = m.engagement_point
        self.agg.ingest_capture(GwCapture(
            drone_id=int(m.drone_id), target_id=int(m.target_id),
            result=int(m.result), enu=np.array([p.x, p.y, p.z]),
            stamp=_stamp(m.header)))

    def _on_clock(self, m):
        self.agg.set_time(float(m.data))

    def _on_log(self, m):
        self.logs.ingest(GwLog(
            stamp=float(m.stamp.sec) + float(m.stamp.nanosec) * 1e-9,
            level=int(m.level), source=str(m.name), msg=str(m.msg),
            file=str(m.file), func=str(m.function), line=int(m.line)))

    # --- scenario proxy ----------------------------------------------------

    def _on_run_status(self, m):
        if self.loop is None or self.broadcast is None:
            return
        status = {
            "type": "status", "channel": "sim",
            "state": int(m.state), "scenarioId": m.scenario_id, "runId": m.run_id,
            "elapsedS": float(m.elapsed_s), "dronesUp": int(m.drones_up),
            "targetsUp": int(m.targets_up),
            "recordingSessionId": m.recording_session_id,
            "requestedRtf": float(m.requested_rtf), "actualRtf": float(m.actual_rtf),
            "rtfControllable": bool(m.rtf_controllable), "message": m.message,
        }
        import asyncio
        asyncio.run_coroutine_threadsafe(self.broadcast(status), self.loop)

    async def scenario_proxy(self, action: str, msg: dict):
        if action == "scenarios.run":
            return await self._run_scenario(msg)
        if action == "scenarios.stop":
            return await self._stop_scenario()
        if action == "sim.setSpeed":
            return await self._set_speed(msg)
        raise RuntimeError(f"unsupported proxy action: {action}")

    async def _run_scenario(self, msg: dict):
        import asyncio
        sc = self.cfg.scenarios
        if not self._action_client.wait_for_server(timeout_sec=2.0):
            raise RuntimeError("orchestrator action server unavailable")
        # drop the prior run's entities so the new run starts from a clean scene
        # (the client clears too, but this stops a stale frame racing the reset)
        self.agg.clear()
        goal = self._RunScenario.Goal()
        goal.scenario_id = str(msg.get("scenarioId", ""))
        goal.overrides_yaml = str(msg.get("overridesYaml", "")) if sc.allow_overrides else ""
        goal.record = bool(msg.get("record", sc.auto_record_on_run))
        goal.requested_rtf = float(msg.get("requestedRtf", sc.default_rtf))
        # fire-and-forget; progress arrives via /sim_run_status relay
        fut = self._action_client.send_goal_async(goal)
        await asyncio.wrap_future(self._as_concurrent(fut))
        return {"accepted": True, "scenarioId": goal.scenario_id}

    async def _stop_scenario(self):
        # canceling the active goal stops the run (orchestrator tears down the launch)
        self._action_client._cancel_all_goals() if hasattr(
            self._action_client, "_cancel_all_goals") else None
        return {"requested": True}

    async def _set_speed(self, msg: dict):
        import asyncio
        if not self.cfg.scenarios.allow_runtime_speed:
            return {"success": False, "message": "runtime speed locked (software-sim profile)"}
        if not self._speed_client.wait_for_service(timeout_sec=2.0):
            raise RuntimeError("set_sim_speed service unavailable")
        req = self._SetSimSpeed.Request()
        req.requested_rtf = float(msg.get("requestedRtf", 1.0))
        fut = self._speed_client.call_async(req)
        resp = await asyncio.wrap_future(self._as_concurrent(fut))
        return {"success": resp.success, "appliedRtf": resp.applied_rtf,
                "message": resp.message}

    @staticmethod
    def _as_concurrent(rclpy_future):
        """Adapt an rclpy Future to a concurrent.futures.Future for asyncio."""
        import concurrent.futures
        cf: concurrent.futures.Future = concurrent.futures.Future()

        def _done(f):
            try:
                cf.set_result(f.result())
            except Exception as e:  # noqa: BLE001 - propagate to awaiter
                cf.set_exception(e)

        rclpy_future.add_done_callback(_done)
        return cf
