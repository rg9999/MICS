"""allocator — GS task allocation + reassignment (PRD §3.2, §7.2).

Maintains the roster from /state_sharing, computes drone->target assignments from
/tracks minimising time-to-intercept, and publishes /assignments. Reassigns on
FAILED/LOST within the configured latency.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

from mics.allocator import Allocator
from mics.config import load_scenario
from mics_msgs.msg import Assignment as AssignmentMsg
from mics_msgs.msg import DroneStatus as DroneStatusMsg
from mics_msgs.msg import TargetTrack as TargetTrackMsg

from . import conversions as cv


class AllocatorNode(Node):
    def __init__(self):
        super().__init__("allocator")
        self.declare_parameter("scenario", "")
        self.declare_parameter("rate_factor", 5.0)
        self.cfg = load_scenario(self.get_parameter("scenario").value)
        rate_factor = float(self.get_parameter("rate_factor").value)
        self.dt = self.cfg.dt
        self.t = 0.0
        self.allocator = Allocator(self.cfg.allocation)
        self._tracks = {}      # target_id -> TargetTrack
        self._track_seen = {}  # target_id -> last wall-tick time seen
        self._status = {}      # drone_id -> DroneStatus
        self.track_ttl = 1.0   # s; drop tracks the GS stopped publishing

        self.create_subscription(TargetTrackMsg, "/tracks", self._on_track, 50)
        self.create_subscription(DroneStatusMsg, "/state_sharing", self._on_status, 50)
        self.create_subscription(Float64, "/sim/clock", self._on_clock, 2000)
        self.pub = self.create_publisher(AssignmentMsg, "/assignments", 10)
        self.get_logger().info(
            f"allocator up: algorithm={self.cfg.allocation.algorithm}, "
            f"redundancy={self.cfg.allocation.redundancy}")

    def _on_track(self, m: TargetTrackMsg):
        tid = int(m.target_id)
        self._tracks[tid] = cv.track_from_msg(m)
        self._track_seen[tid] = self.t

    def _on_status(self, m: DroneStatusMsg):
        self._status[int(m.drone_id)] = cv.status_from_msg(m)

    def _on_clock(self, m: Float64):
        self.t = m.data
        # expire tracks the GS no longer publishes (target captured / aged out)
        for tid in [t for t, ts in self._track_seen.items()
                    if self.t - ts > self.track_ttl]:
            self._tracks.pop(tid, None)
            self._track_seen.pop(tid, None)
        self.allocator.update_roster(list(self._status.values()))
        assignments = self.allocator.allocate(self.t, list(self._tracks.values()))
        for a in assignments.values():
            self.pub.publish(cv.assignment_to_msg(a))


def main(argv=None):
    rclpy.init(args=argv)
    node = AllocatorNode()
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
