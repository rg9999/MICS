"""attacker_sim — internal target source (PRD §5.2).

Steps the scripted attackers and publishes ground-truth tracks on /sim/truth
(consumed only by onboard sensor models + monitor, never by autonomy directly).
Owns truth, so it is the authority that validates captures and kills targets.
"""

from __future__ import annotations

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64, Int32

from mics.attacker import Attacker
from mics.config import load_scenario
from std_msgs.msg import Float64

from mics.geometry import norm
from mics.msgs import CaptureResult, TrackSource
from mics_msgs.msg import CaptureEvent as CaptureEventMsg
from mics_msgs.msg import DroneStatus as DroneStatusMsg
from mics_msgs.msg import TargetTrack as TargetTrackMsg

from . import conversions as cv


class AttackerSimNode(Node):
    def __init__(self):
        super().__init__("attacker_sim")
        self.declare_parameter("scenario", "")
        self.declare_parameter("rate_factor", 5.0)
        path = self.get_parameter("scenario").value
        self.cfg = load_scenario(path)
        rate_factor = float(self.get_parameter("rate_factor").value)
        self.dt = self.cfg.dt
        self.t = 0.0
        self.r_capture = self.cfg.drone.r_capture_m

        self.attackers = []
        for ac in self.cfg.attackers:
            self.attackers.append(Attacker(
                target_id=ac.id, profile=ac.profile, speed_mps=ac.speed_mps,
                position=np.array(ac.start, dtype=float),
                asset=np.array(ac.asset, dtype=float),
                waypoints=[np.array(w, dtype=float) for w in ac.waypoints],
                r_evade_m=ac.r_evade_m,
                rng=np.random.default_rng(self.cfg.seed + 100 + ac.id),
            ))

        self.defender_pos = {}
        # attacker_sim is the world authority, so it also owns the sim clock:
        # every other node steps in lockstep off /sim/clock rather than a
        # free-running local timer (which desynchronises badly under load).
        # deep queue so no clock tick is ever dropped: lagging subscribers stay
        # numerically in lockstep (they just fall behind in wall time)
        self.clock_pub = self.create_publisher(Float64, "/sim/clock", 2000)
        self.truth_pub = self.create_publisher(TargetTrackMsg, "/sim/truth", 10)
        self.alive_pub = self.create_publisher(Int32, "/sim/alive_count", 10)
        self.done_pub = self.create_publisher(Bool, "/sim/done", 10)
        self.create_subscription(DroneStatusMsg, "/state_sharing", self._on_status, 50)
        self.create_subscription(CaptureEventMsg, "/capture_events", self._on_capture, 50)

        self.timer = self.create_timer(self.dt / rate_factor, self._tick)
        self._done_sent = False
        self.get_logger().info(
            f"attacker_sim up: {len(self.attackers)} target(s), dt={self.dt}, "
            f"{rate_factor:.0f}x wall, r_capture={self.r_capture}m")

    def _on_status(self, m: DroneStatusMsg):
        self.defender_pos[int(m.drone_id)] = cv._arr(m.pose.position)

    def _on_capture(self, m: CaptureEventMsg):
        if int(m.result) != int(CaptureResult.SUCCESS):
            return
        ep = cv._arr(m.engagement_point)
        for a in self.attackers:
            if a.target_id == int(m.target_id) and a.alive:
                if norm(a.position - ep) <= self.r_capture + 0.5:
                    a.kill()
                    self.get_logger().info(
                        f"target {a.target_id} CAPTURED by drone {int(m.drone_id)}")

    def _tick(self):
        self.t += self.dt
        self.clock_pub.publish(Float64(data=self.t))
        dpos = list(self.defender_pos.values())
        for a in self.attackers:
            a.step(self.dt, dpos)
        alive = 0
        for a in self.attackers:
            if not a.alive:
                continue
            alive += 1
            st = a.state()
            from mics.msgs import TargetTrack
            trk = TargetTrack(stamp=self.t, target_id=a.target_id,
                              position=st.position, velocity=st.velocity,
                              source=TrackSource.INTERNAL_SIM, has_velocity=True)
            self.truth_pub.publish(cv.track_to_msg(trk))
        self.alive_pub.publish(Int32(data=alive))
        if alive == 0 and not self._done_sent:
            self._done_sent = True
            self.done_pub.publish(Bool(data=True))
            self.get_logger().info("all targets neutralised — /sim/done")


def main(argv=None):
    rclpy.init(args=argv)
    node = AttackerSimNode()
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
