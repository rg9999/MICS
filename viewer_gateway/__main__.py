"""Gateway entry point: ``python -m viewer_gateway --source fixture``.

Picks one of the three state sources and starts the WebSocket server. Only the
``ros`` source requires a ROS2 environment; ``fixture`` and ``replay`` run on a
bare Python host, which is what the dev/frontend workflow and the smoke tests
rely on.
"""

from __future__ import annotations

import argparse
import asyncio
import sys

from .aggregator import Aggregator
from .config import load_config
from .geodesy import Transformer
from .logbus import LogBus
from .recorder import Recorder
from .server import GatewayServer
from .sources import FixtureSource, ReplaySource


def _build_source(name: str, args, cfg, agg, logbus):
    if name == "fixture":
        return FixtureSource(agg, logbus)
    if name == "replay":
        if not args.session:
            sys.exit("--session DIR is required for --source replay")
        return ReplaySource(args.session)
    if name == "ros":
        from .ros_source import RosSource  # lazy: pulls in rclpy
        return RosSource(cfg, agg, logbus)
    sys.exit(f"unknown source: {name}")


def main(argv=None):
    ap = argparse.ArgumentParser(prog="viewer_gateway")
    ap.add_argument("--source", default="fixture", choices=["fixture", "replay", "ros"])
    ap.add_argument("--config", default=None, help="gateway YAML config path")
    ap.add_argument("--session", default=None, help="recording dir for --source replay")
    ap.add_argument("--host", default=None)
    ap.add_argument("--port", type=int, default=None)
    args = ap.parse_args(argv)

    cfg = load_config(args.config)
    if args.host is not None:
        cfg.ws_host = args.host
    if args.port is not None:
        cfg.ws_port = args.port

    agg = Aggregator(Transformer(cfg.datum))
    logbus = LogBus(min_level=cfg.log_min_level, ring_rows=cfg.log_ring_buffer_rows)
    recorder = Recorder(cfg.recording, datum={"lat": cfg.datum.lat,
                                              "lon": cfg.datum.lon,
                                              "alt": cfg.datum.alt})
    source = _build_source(args.source, args, cfg, agg, logbus)
    server = GatewayServer(cfg, agg, logbus, recorder, source)

    print(f"viewer-gateway: source={args.source} ws://{cfg.ws_host}:{cfg.ws_port}",
          file=sys.stderr)
    try:
        asyncio.run(server.serve())
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
