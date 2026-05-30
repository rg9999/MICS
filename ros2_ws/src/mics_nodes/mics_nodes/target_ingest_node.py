"""target_ingest — GS source normaliser + cue degrader (PRD §3.1, §5.5).

Subscribes ground truth on /sim/truth (internal source), degrades it (noise,
dropout, latency) into partial cues, and republishes on /tracks_raw. In external
mode this is where a UDP/JSON or Zenoh bridge would feed instead.
"""

from __future__ import annotations

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

from mics.attacker import AttackerState
from mics.config import load_scenario
from mics.cue_degrader import CueDegrader
from mics_msgs.msg import TargetTrack as TargetTrackMsg

from . import conversions as cv


class TargetIngestNode(Node):
    def __init__(self):
        super().__init__("target_ingest")
        self.declare_parameter("scenario", "")
        self.declare_parameter("rate_factor", 5.0)
        self.cfg = load_scenario(self.get_parameter("scenario").value)
        rate_factor = float(self.get_parameter("rate_factor").value)
        self.dt = self.cfg.dt
        self.t = 0.0
        self.degrader = CueDegrader(
            pos_sigma_m=self.cfg.cue_pos_sigma_m,
            dropout_pct=self.cfg.cue_dropout_pct,
            latency_ms=self.cfg.cue_latency_ms,
            rng=np.random.default_rng(self.cfg.seed + 1),
        )
        self._truth = {}       # target_id -> AttackerState
        self._truth_seen = {}  # target_id -> last clock time seen
        self.truth_ttl = 0.5   # s; attacker_sim stops publishing killed targets
        self.create_subscription(TargetTrackMsg, "/sim/truth", self._on_truth, 50)
        self.create_subscription(Float64, "/sim/clock", self._on_clock, 2000)
        self.pub = self.create_publisher(TargetTrackMsg, "/tracks_raw", 10)
        self.get_logger().info(
            f"target_ingest up: pos_sigma={self.cfg.cue_pos_sigma_m}m, "
            f"dropout={self.cfg.cue_dropout_pct}%, latency={self.cfg.cue_latency_ms}ms")

    def _on_truth(self, m: TargetTrackMsg):
        tid = int(m.target_id)
        self._truth[tid] = AttackerState(
            tid, cv._arr(m.position), cv._arr(m.velocity), True)
        self._truth_seen[tid] = self.t

    def _on_clock(self, m: Float64):
        self.t = m.data
        # drop targets attacker_sim no longer publishes (killed) so we stop
        # degrading a ghost into a phantom cue
        for tid in [t for t, ts in self._truth_seen.items()
                    if self.t - ts > self.truth_ttl]:
            self._truth.pop(tid, None)
            self._truth_seen.pop(tid, None)
        self.degrader.ingest(self.t, list(self._truth.values()))
        for trk in self.degrader.release(self.t):
            self.pub.publish(cv.track_to_msg(trk))


def main(argv=None):
    rclpy.init(args=argv)
    node = TargetIngestNode()
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
