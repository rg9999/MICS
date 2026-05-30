"""Always-on orchestrator: serves run/stop + status without launching a scenario.

Unlike mics.launch.py (which *is* one scenario run), this brings up only the
mics_sim_orchestrator. It then launches/tears down scenario runs on demand via
the RunScenario action, so a viewer-gateway can drive the sim from the browser.

Usage:
  ros2 launch mics_nodes orchestrator.launch.py catalog_dir:=/scenarios
"""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("catalog_dir", default_value="/scenarios"),
        DeclareLaunchArgument("default_rtf", default_value="2.0"),
        Node(
            package="mics_nodes", executable="sim_orchestrator",
            name="mics_sim_orchestrator", output="screen",
            parameters=[{
                "catalog_dir": LaunchConfiguration("catalog_dir"),
                "default_rtf": LaunchConfiguration("default_rtf"),
                "rtf_controllable": False,
            }],
        ),
    ])
