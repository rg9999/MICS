"""Always-on orchestrator: serves run/stop + status without launching a scenario.

Unlike mics.launch.py (which *is* one scenario run), this brings up only the
mics_sim_orchestrator. It then launches/tears down scenario runs on demand via
the RunScenario action, so a viewer-gateway can drive the sim from the browser.

The same node drives either backend: the software-sim topology (mics.launch.py,
fixed RTF) or the Gazebo topology (gazebo.launch.py, live RTF). Switch with the
launch_file/speed_arg/rtf_controllable args.

Usage:
  ros2 launch mics_nodes orchestrator.launch.py catalog_dir:=/scenarios
  ros2 launch mics_nodes orchestrator.launch.py launch_file:=gazebo.launch.py \
      speed_arg:=rtf rtf_controllable:=true
"""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("catalog_dir", default_value="/scenarios"),
        DeclareLaunchArgument("default_rtf", default_value="2.0"),
        # Software-sim defaults; the gazebo-viewer profile overrides these.
        DeclareLaunchArgument("launch_file", default_value="mics.launch.py"),
        DeclareLaunchArgument("speed_arg", default_value="rate_factor"),
        DeclareLaunchArgument("gz_world", default_value="mics"),
        DeclareLaunchArgument("rtf_controllable", default_value="false"),
        Node(
            package="mics_nodes", executable="sim_orchestrator",
            name="mics_sim_orchestrator", output="screen",
            parameters=[{
                "catalog_dir": LaunchConfiguration("catalog_dir"),
                "default_rtf": ParameterValue(
                    LaunchConfiguration("default_rtf"), value_type=float),
                "launch_file": LaunchConfiguration("launch_file"),
                "speed_arg": LaunchConfiguration("speed_arg"),
                "gz_world": LaunchConfiguration("gz_world"),
                "rtf_controllable": ParameterValue(
                    LaunchConfiguration("rtf_controllable"), value_type=bool),
            }],
        ),
    ])
