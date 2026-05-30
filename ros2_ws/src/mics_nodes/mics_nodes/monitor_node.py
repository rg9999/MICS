"""monitor — scoring + run summary (PRD §9.3).

Passive observer: tallies captures (truth-validated), false-fires, misses,
handoffs, reassignments, and min inter-drone separation. Prints the metrics
summary and shuts the run down on /sim/done (all targets neutralised) or when
max_time is reached.
"""

from __future__ import annotations

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64

from mics.config import load_scenario
from mics.geometry import norm
from mics.msgs import CaptureResult, DroneState
from mics_msgs.msg import Assignment as AssignmentMsg
from mics_msgs.msg import CaptureEvent as CaptureEventMsg
from mics_msgs.msg import DroneStatus as DroneStatusMsg
from mics_msgs.msg import TargetTrack as TargetTrackMsg

from . import conversions as cv


class MonitorNode(Node):
    def __init__(self):
        super().__init__("monitor")
        self.declare_parameter("scenario", "")
        self.declare_parameter("rate_factor", 5.0)
        self.cfg = load_scenario(self.get_parameter("scenario").value)
        rate_factor = float(self.get_parameter("rate_factor").value)
        self.dt = self.cfg.dt
        self.t = 0.0
        self.r_cap = self.cfg.drone.r_capture_m
        self.n_targets = len(self.cfg.attackers)

        self.captures = 0
        self.misses = 0
        self.false_fires = 0
        self.reassignments = 0
        self.tti = []
        self.min_sep = float("inf")
        self._handoff_seen = set()
        self._tti_start = {}              # target_id -> first-assigned time
        self._prev_target = {}            # drone_id -> last target_id
        self._truth = {}                  # target_id -> position
        self._air = {}                    # drone_id -> position (airborne only)
        self._captured = set()
        self._done = False

        self.create_subscription(CaptureEventMsg, "/capture_events", self._on_capture, 50)
        self.create_subscription(TargetTrackMsg, "/sim/truth", self._on_truth, 50)
        self.create_subscription(DroneStatusMsg, "/state_sharing", self._on_status, 50)
        self.create_subscription(AssignmentMsg, "/assignments", self._on_assign, 50)
        self.create_subscription(Bool, "/sim/done", self._on_done, 10)
        self.create_subscription(Float64, "/sim/clock", self._on_clock, 2000)
        self.get_logger().info(f"monitor up: {self.n_targets} target(s)")

    def _on_truth(self, m: TargetTrackMsg):
        self._truth[int(m.target_id)] = cv._arr(m.position)

    def _on_assign(self, m: AssignmentMsg):
        did, tid = int(m.drone_id), int(m.target_id)
        prev = self._prev_target.get(did, 0)
        if prev != tid and tid != 0 and prev != 0:
            self.reassignments += 1
        self._prev_target[did] = tid
        if tid != 0 and tid not in self._tti_start:
            self._tti_start[tid] = self.t

    def _on_status(self, m: DroneStatusMsg):
        did = int(m.drone_id)
        state = DroneState(int(m.state))
        if state == DroneState.TERMINAL and did not in self._handoff_seen:
            self._handoff_seen.add(did)
        if state in (DroneState.MIDCOURSE, DroneState.ACQUIRING, DroneState.TERMINAL):
            self._air[did] = cv._arr(m.pose.position)
        else:
            self._air.pop(did, None)

    def _on_capture(self, m: CaptureEventMsg):
        if int(m.result) == int(CaptureResult.SUCCESS):
            tid = int(m.target_id)
            ep = cv._arr(m.engagement_point)
            tpos = self._truth.get(tid)
            if tpos is not None and norm(tpos - ep) <= self.r_cap + 0.5 and tid not in self._captured:
                self._captured.add(tid)
                self.captures += 1
                if tid in self._tti_start:
                    self.tti.append(self.t - self._tti_start[tid])
            else:
                self.false_fires += 1
        elif int(m.result) == int(CaptureResult.MISS):
            self.misses += 1

    def _on_done(self, m: Bool):
        if m.data and not self._done:
            self._finish("all targets neutralised")

    def _on_clock(self, m: Float64):
        self.t = m.data
        air = list(self._air.values())
        for i in range(len(air)):
            for j in range(i + 1, len(air)):
                self.min_sep = min(self.min_sep, norm(air[i] - air[j]))
        if not self._done and self.t >= self.cfg.max_time_s:
            self._finish("max_time reached")

    def _summary(self) -> dict:
        n = max(self.n_targets, 1)
        mtti = float(sum(self.tti) / len(self.tti)) if self.tti else None
        return {
            "capture_success_rate": self.captures / n,
            "captures": self.captures,
            "misses": self.misses,
            "n_targets": self.n_targets,
            "mean_time_to_intercept_s": (round(mtti, 2) if mtti is not None else None),
            "reassignments": self.reassignments,
            "min_inter_drone_separation_m": (None if self.min_sep == float("inf")
                                             else round(self.min_sep, 2)),
            "false_fires": self.false_fires,
            "handoffs": len(self._handoff_seen),
            "sim_time_s": round(self.t, 2),
        }

    def _finish(self, reason: str) -> None:
        self._done = True
        self.get_logger().info(f"RUN COMPLETE ({reason})")
        self.get_logger().info("METRICS " + json.dumps(self._summary()))
        # Raise out of the callback to break spin() reliably (calling
        # rclpy.shutdown() from inside a callback does not). The monitor
        # process then exits, which the launch file's OnProcessExit handler
        # turns into a full-launch Shutdown so the container tears down.
        raise SystemExit(0)


def main(argv=None):
    rclpy.init(args=argv)
    node = MonitorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
