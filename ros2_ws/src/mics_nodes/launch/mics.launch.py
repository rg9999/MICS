"""MICS bring-up: ground station + attacker sim + N aircraft + monitor.

The number of aircraft is read from the scenario YAML (defenders.count) so a
single launch arg drives the whole topology. All nodes share scenario and
rate_factor so they advance their internal clocks in lockstep.

Usage:
  ros2 launch mics_nodes mics.launch.py scenario:=/scenarios/happy_path.yaml
"""

from __future__ import annotations

import yaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _spawn(context, *args, **kwargs):
    scenario = LaunchConfiguration("scenario").perform(context)
    rate_factor = LaunchConfiguration("rate_factor").perform(context)

    with open(scenario) as f:
        data = yaml.safe_load(f)
    n = int((data.get("defenders", {}) or {}).get("count", 4))

    params = [{"scenario": scenario, "rate_factor": float(rate_factor)}]

    monitor = Node(package="mics_nodes", executable="monitor",
                   name="monitor", output="screen", parameters=params)

    nodes = [
        Node(package="mics_nodes", executable="attacker_sim",
             name="attacker_sim", output="screen", parameters=params),
        Node(package="mics_nodes", executable="target_ingest",
             name="target_ingest", output="screen", parameters=params),
        Node(package="mics_nodes", executable="track_manager",
             name="track_manager", output="screen", parameters=params),
        Node(package="mics_nodes", executable="allocator",
             name="allocator", output="screen", parameters=params),
        monitor,
        # when the monitor exits (run scored), tear the whole launch down
        RegisterEventHandler(OnProcessExit(
            target_action=monitor,
            on_exit=[EmitEvent(event=Shutdown(reason="run complete"))])),
    ]
    for did in range(1, n + 1):
        nodes.append(Node(
            package="mics_nodes", executable="aircraft",
            name=f"aircraft_{did}", output="screen",
            parameters=params + [{"drone_id": did}]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("scenario", default_value="/scenarios/happy_path.yaml"),
        # 2x is the sustainable faster-than-real-time rate for this headless,
        # single-container, software-timed topology on a CPU-only host. Higher
        # rates saturate the single-threaded executors / DDS and start dropping
        # the depth-bounded data topics, so the pipeline stops engaging.
        DeclareLaunchArgument("rate_factor", default_value="2.0"),
        OpaqueFunction(function=_spawn),
    ])
