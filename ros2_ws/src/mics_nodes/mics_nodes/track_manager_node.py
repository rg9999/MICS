"""track_manager — GS track fusion + TTL (PRD §3.1, §7.1).

Associates/ages/fuses partial cues from /tracks_raw and publishes the fused
picture on /tracks at >=5 Hz.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

from mics.config import load_scenario
from mics.track_manager import TrackManager
from mics_msgs.msg import TargetTrack as TargetTrackMsg

from . import conversions as cv


class TrackManagerNode(Node):
    def __init__(self):
        super().__init__("track_manager")
        self.declare_parameter("scenario", "")
        self.declare_parameter("rate_factor", 5.0)
        self.cfg = load_scenario(self.get_parameter("scenario").value)
        rate_factor = float(self.get_parameter("rate_factor").value)
        self.dt = self.cfg.dt
        self.t = 0.0
        self.tm = TrackManager()
        self.create_subscription(TargetTrackMsg, "/tracks_raw", self._on_raw, 50)
        self.create_subscription(Float64, "/sim/clock", self._on_clock, 2000)
        self.pub = self.create_publisher(TargetTrackMsg, "/tracks", 10)
        self.get_logger().info("track_manager up")

    def _on_raw(self, m: TargetTrackMsg):
        self.tm.ingest(self.t, [cv.track_from_msg(m)])

    def _on_clock(self, m: Float64):
        self.t = m.data
        self.tm.step(self.t)
        for trk in self.tm.tracks():
            self.pub.publish(cv.track_to_msg(trk))


def main(argv=None):
    rclpy.init(args=argv)
    node = TrackManagerNode()
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
