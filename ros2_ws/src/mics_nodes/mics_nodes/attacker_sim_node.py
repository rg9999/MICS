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
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

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
        self.declare_parameter("profile", "software-sim")
        self.profile = self.get_parameter("profile").value
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
        self.truth_pub = self.create_publisher(TargetTrackMsg, "/sim/truth", 10)
        self.alive_pub = self.create_publisher(Int32, "/sim/alive_count", 10)
        self.done_pub = self.create_publisher(Bool, "/sim/done", 10)
        self.create_subscription(DroneStatusMsg, "/state_sharing", self._on_status, 50)
        self.create_subscription(CaptureEventMsg, "/capture_events", self._on_capture, 50)
        self._done_sent = False

        if self.profile == "gazebo":
            # Gazebo owns physics + the clock (bridged to /sim/clock). We drive
            # the scripts off that tick and command each target's velocity to its
            # model; truth comes from the model odometry, not internal integration.
            self._aodom = {}        # target_id -> (pos, vel) from Gazebo
            self.cmd_pubs = {}
            for a in self.attackers:
                name = f"attacker_{a.target_id}"
                self.cmd_pubs[a.target_id] = self.create_publisher(
                    Twist, f"/model/{name}/cmd_vel", 10)
                self.create_subscription(
                    Odometry, f"/model/{name}/odometry",
                    lambda m, tid=a.target_id: self._on_attacker_odom(m, tid), 50)
            self.create_subscription(Float64, "/sim/clock", self._on_clock_tick, 2000)
        else:
            # software-sim: attacker_sim is the world authority and owns the sim
            # clock. Every other node steps in lockstep off /sim/clock rather than
            # a free-running local timer (which desynchronises badly under load).
            # Deep queue so no tick is dropped: lagging subscribers stay
            # numerically in lockstep (they just fall behind in wall time).
            self.clock_pub = self.create_publisher(Float64, "/sim/clock", 2000)
            self.timer = self.create_timer(self.dt / rate_factor, self._tick)
        self.get_logger().info(
            f"attacker_sim up: {len(self.attackers)} target(s), dt={self.dt}, "
            f"profile={self.profile}, r_capture={self.r_capture}m")

    def _on_attacker_odom(self, m: Odometry, tid: int):
        p = m.pose.pose.position
        v = m.twist.twist.linear
        self._aodom[tid] = (np.array([p.x, p.y, p.z]), np.array([v.x, v.y, v.z]))

    def _on_clock_tick(self, m: Float64):
        """gazebo profile: advance scripts off the bridged clock and command
        each target's velocity; publish ground truth from the model odometry."""
        from mics.msgs import TargetTrack
        self.t = m.data
        dpos = list(self.defender_pos.values())
        alive = 0
        for a in self.attackers:
            if not a.alive:
                continue
            alive += 1
            od = self._aodom.get(a.target_id)
            if od is not None:
                a.position = od[0]          # script reasons about the true pose
            a.step(self.dt, dpos)           # advance heading (internal pos ignored)
            cmd = a.state().velocity
            tw = Twist()
            tw.linear.x, tw.linear.y, tw.linear.z = (float(cmd[0]), float(cmd[1]), float(cmd[2]))
            self.cmd_pubs[a.target_id].publish(tw)
            pos = od[0] if od is not None else a.position
            vel = od[1] if od is not None else cmd
            trk = TargetTrack(stamp=self.t, target_id=a.target_id, position=pos,
                              velocity=vel, source=TrackSource.INTERNAL_SIM,
                              has_velocity=True)
            self.truth_pub.publish(cv.track_to_msg(trk))
        self.alive_pub.publish(Int32(data=alive))
        if alive == 0 and not self._done_sent:
            self._done_sent = True
            self.done_pub.publish(Bool(data=True))
            self.get_logger().info("all targets neutralised — /sim/done")

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
