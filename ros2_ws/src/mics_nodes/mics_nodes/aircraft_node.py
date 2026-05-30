"""aircraft — per-drone onboard autonomy (PRD §3.3, §6).

Owns one mics.drone.Drone (FSM + fusion + guidance + safety + kinematics).
Subscribes its own /assignments, the fused /tracks (GS cue), /sim/truth (feeds
the onboard sensor models), and /state_sharing (teammates for capture interlock).
Publishes its own DroneStatus on /state_sharing and any /capture_events.
"""

from __future__ import annotations

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

from mics.attacker import AttackerState
from mics.config import load_scenario
from mics.drone import Drone
from mics.msgs import DroneState
from mics.sensors import SensorSuite
from mics_msgs.msg import Assignment as AssignmentMsg
from mics_msgs.msg import CaptureEvent as CaptureEventMsg
from mics_msgs.msg import DroneStatus as DroneStatusMsg
from mics_msgs.msg import TargetTrack as TargetTrackMsg

from . import conversions as cv


class AircraftNode(Node):
    def __init__(self):
        super().__init__("aircraft")
        self.declare_parameter("scenario", "")
        self.declare_parameter("rate_factor", 5.0)
        self.declare_parameter("drone_id", 1)
        self.cfg = load_scenario(self.get_parameter("scenario").value)
        rate_factor = float(self.get_parameter("rate_factor").value)
        self.dt = self.cfg.dt
        self.t = 0.0
        did = int(self.get_parameter("drone_id").value)

        n = self.cfg.n_defenders
        x = (did - 1 - (n - 1) / 2) * self.cfg.defender_spacing_m
        self.drone = Drone(
            drone_id=did,
            position=np.array([x, -50.0, 50.0]),
            params=self.cfg.drone,
            safety_cfg=self.cfg.safety,
            rng=np.random.default_rng(self.cfg.seed + 200 + did),
        )
        self.drone.sensors = SensorSuite(
            mode=self.cfg.sensor_mode, rng=self.drone.rng,
            camera_enabled=self.cfg.camera_enabled,
            radar_enabled=self.cfg.radar_enabled,
            lidar_enabled=self.cfg.lidar_enabled,
        )

        self._tracks = {}      # target_id -> TargetTrack (fused cue)
        self._truth = {}       # target_id -> AttackerState (sensor input)
        self._truth_seen = {}  # target_id -> last clock time seen
        self.truth_ttl = 0.5   # s; killed targets stop publishing truth
        self._teammates = {}   # drone_id -> DroneStatus

        self.create_subscription(AssignmentMsg, "/assignments", self._on_assign, 50)
        self.create_subscription(TargetTrackMsg, "/tracks", self._on_track, 50)
        self.create_subscription(TargetTrackMsg, "/sim/truth", self._on_truth, 50)
        self.create_subscription(DroneStatusMsg, "/state_sharing", self._on_status, 50)
        self.create_subscription(Float64, "/sim/clock", self._on_clock, 2000)
        self.status_pub = self.create_publisher(DroneStatusMsg, "/state_sharing", 10)
        self.capture_pub = self.create_publisher(CaptureEventMsg, "/capture_events", 10)
        self.get_logger().info(f"aircraft {did} up at x={x:.0f}")

    def _on_assign(self, m: AssignmentMsg):
        if int(m.drone_id) != self.drone.drone_id:
            return
        self.drone.set_assignment(cv.assignment_from_msg(m))

    def _on_track(self, m: TargetTrackMsg):
        self._tracks[int(m.target_id)] = cv.track_from_msg(m)

    def _on_truth(self, m: TargetTrackMsg):
        tid = int(m.target_id)
        self._truth[tid] = AttackerState(
            tid, cv._arr(m.position), cv._arr(m.velocity), True)
        self._truth_seen[tid] = self.t

    def _on_status(self, m: DroneStatusMsg):
        did = int(m.drone_id)
        if did != self.drone.drone_id:
            self._teammates[did] = cv.status_from_msg(m)

    def _on_clock(self, m: Float64):
        self.t = m.data
        now = self.t

        for tid in [t for t, ts in self._truth_seen.items()
                    if self.t - ts > self.truth_ttl]:
            self._truth.pop(tid, None)
            self._truth_seen.pop(tid, None)

        cue = None
        atruth = None
        a = self.drone.assignment
        if a is not None and a.target_id != 0:
            cue = self._tracks.get(a.target_id)
            atruth = self._truth.get(a.target_id)

        self.drone.step(now, self.dt, cue, atruth, list(self._teammates.values()))

        for ev in self.drone.capture_events:
            reason = getattr(self.drone, "fail_reason", None)
            self.get_logger().info(
                f"event t={now:.2f} result={int(ev.result)} reason={reason} "
                f"target={ev.target_id} pos={self.drone.position.round(1)} "
                f"batt={self.drone.battery_pct:.0f}")
            self.capture_pub.publish(cv.capture_to_msg(ev))
        self.drone.capture_events.clear()

        self.status_pub.publish(cv.status_to_msg(self.drone.status(now)))


def main(argv=None):
    rclpy.init(args=argv)
    node = AircraftNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
