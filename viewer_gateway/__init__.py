"""MICS viewer-gateway — the choke point between the ROS2/Zenoh bus and the
browser (PRD_viewer_architecture.md §6.2).

Responsibilities (this package):
  * subscribe to the MICS bus and coalesce into per-frame ``FrameSnapshot``
  * transform ENU (sim/local) -> geodetic (WGS84) once, server-side
  * compute grid-derived fields (range-to-target, allocation status, speed, ...)
  * aggregate ``/rosout`` into a bounded, level-throttled ``LogBatch`` channel
  * record the live stream to disk and replay it over the same socket
  * serve the scenario catalog and proxy run/stop to mics_sim_orchestrator
  * be the single auth boundary when operator controls are enabled

The pure-logic modules (config, geodesy, aggregator, snapshot, recorder,
replayer, catalog) import no rclpy, so they are unit-testable without ROS.
``ros_source`` is the only module that needs a live ROS2 graph.
"""

__version__ = "0.1.0"
