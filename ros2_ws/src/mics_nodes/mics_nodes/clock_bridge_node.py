"""clock_bridge — Gazebo /clock -> MICS /sim/clock (gazebo profile only).

In the software-sim profile attacker_sim owns the clock. In the gazebo profile
Gazebo is the fixed-step time authority: ros_gz_bridge republishes Gazebo's
sim time on /clock (rosgraph_msgs/Clock), and this node re-emits it as the
Float64 /sim/clock ticks every MICS node already steps off.

Gazebo may publish /clock faster (or slower) than the control dt, so we don't
forward 1:1. Instead we emit one monotonic tick per `dt` of simulated time
crossed, which gives the downstream nodes exactly the cadence they expect.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float64

from mics.config import load_scenario


class ClockBridgeNode(Node):
    def __init__(self):
        super().__init__("clock_bridge")
        self.declare_parameter("scenario", "")
        self.declare_parameter("dt", 0.05)
        path = self.get_parameter("scenario").value
        self.dt = load_scenario(path).dt if path else float(self.get_parameter("dt").value)
        self._next = 0.0
        self.pub = self.create_publisher(Float64, "/sim/clock", 2000)
        self.create_subscription(Clock, "/clock", self._on_clock, 100)
        self.get_logger().info(f"clock_bridge up: emitting /sim/clock every dt={self.dt}s")

    def _on_clock(self, m: Clock):
        t = m.clock.sec + m.clock.nanosec * 1e-9
        # emit every dt boundary crossed since the last tick (monotonic, gap-free)
        while t + 1e-9 >= self._next:
            self.pub.publish(Float64(data=self._next))
            self._next += self.dt


def main(argv=None):
    rclpy.init(args=argv)
    node = ClockBridgeNode()
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
