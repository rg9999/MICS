"""MICS — Multi-Drone Cooperative Interception System (pure-Python core simulation).

This package implements the behavior-defining core of MICS (see PRD §4.5): the
coordination, autonomy, fusion, and simulation logic that the PRD designates as
CPU-only and laptop-developable. It deliberately has no ROS2/Gazebo dependency so
it runs and is testable on any host with Python + numpy. Module names mirror the
ROS2 package layout in the PRD so this maps 1:1 onto the real stack later.
"""

__version__ = "0.1.0"
