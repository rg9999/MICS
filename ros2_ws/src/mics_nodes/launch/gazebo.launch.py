"""MICS bring-up on Gazebo Fortress (physics-authoritative, lockstep).

Unlike mics.launch.py (where attacker_sim owns a software clock), here Gazebo is
the fixed-step time authority. The flow is:

  1. ign gazebo -s -r  starts the headless server stepping the `mics` world at
     dt (max_step_size), publishing sim time on ignition's /clock.
  2. ros_gz_bridge republishes /clock (-> rosgraph_msgs/Clock) plus, per model,
     the cmd_vel (ROS->ign) and odometry (ign->ROS) topics the MICS nodes use.
  3. clock_bridge re-emits /clock as the Float64 /sim/clock every MICS node
     already steps off — so the GS + autonomy code runs unchanged.
  4. attacker_sim and each aircraft run with profile:=gazebo: they command model
     velocity and read pose/twist back from odometry instead of integrating.

Entities are spawned at their scenario positions; defenders use the same line
formation as aircraft_node (x = (did-1-(n-1)/2)*spacing, y=-50, z=50).

Usage:
  ros2 launch mics_nodes gazebo.launch.py scenario:=/scenarios/happy_path.yaml
"""

from __future__ import annotations

import os

import yaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _defender_xyz(did: int, n: int, spacing: float):
    x = (did - 1 - (n - 1) / 2.0) * spacing
    return x, -50.0, 50.0


def _spawn(context, *args, **kwargs):
    scenario = LaunchConfiguration("scenario").perform(context)
    gz_dir = LaunchConfiguration("gz_dir").perform(context)
    rtf = LaunchConfiguration("rtf").perform(context)
    world_file = os.path.join(gz_dir, "worlds", "mics.sdf")
    model_file = os.path.join(gz_dir, "models", "vehicle", "model.sdf")

    with open(scenario) as f:
        data = yaml.safe_load(f)
    d = data.get("defenders", {}) or {}
    n = int(d.get("count", 4))
    spacing = 20.0  # matches ScenarioConfig.defender_spacing_m / aircraft_node
    attackers = data.get("attackers", []) or [{"id": 1, "start": [-300.0, 0.0, 50.0]}]

    params = [{"scenario": scenario, "profile": "gazebo"}]

    # 1. headless physics server, started paused-then-run (-r) at world dt
    gz_server = ExecuteProcess(
        cmd=["ign", "gazebo", "-s", "-r", "-v", "2", world_file],
        output="screen",
        additional_env={"IGN_GAZEBO_RESOURCE_PATH": os.path.join(gz_dir, "models")},
    )

    # 2. bridge: /clock (ign->ROS) plus per-model cmd_vel (ROS->ign) + odometry
    bridge_args = ["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"]
    spawns = []

    def _add_entity(name, x, y, z):
        bridge_args.append(
            f"/model/{name}/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist")
        bridge_args.append(
            f"/model/{name}/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry")
        spawns.append(Node(
            package="ros_gz_sim", executable="create", output="screen",
            arguments=["-world", "mics", "-file", model_file, "-name", name,
                       "-x", str(x), "-y", str(y), "-z", str(z)]))

    for did in range(1, n + 1):
        x, y, z = _defender_xyz(did, n, spacing)
        _add_entity(f"defender_{did}", x, y, z)
    for a in attackers:
        s = a.get("start", [-300.0, 0.0, 50.0])
        _add_entity(f"attacker_{int(a.get('id', 1))}", s[0], s[1], s[2])

    bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge",
        name="ros_gz_bridge", output="screen", arguments=bridge_args)

    # 3. clock_bridge: ign /clock -> Float64 /sim/clock at dt cadence
    clock_bridge = Node(
        package="mics_nodes", executable="clock_bridge",
        name="clock_bridge", output="screen", parameters=params)

    # 4. MICS autonomy + GS, all stepping off /sim/clock
    monitor = Node(package="mics_nodes", executable="monitor",
                   name="monitor", output="screen", parameters=params)
    mics_nodes = [
        Node(package="mics_nodes", executable="attacker_sim",
             name="attacker_sim", output="screen", parameters=params),
        Node(package="mics_nodes", executable="target_ingest",
             name="target_ingest", output="screen", parameters=params),
        Node(package="mics_nodes", executable="track_manager",
             name="track_manager", output="screen", parameters=params),
        Node(package="mics_nodes", executable="allocator",
             name="allocator", output="screen", parameters=params),
        monitor,
        RegisterEventHandler(OnProcessExit(
            target_action=monitor,
            on_exit=[EmitEvent(event=Shutdown(reason="run complete"))])),
    ]
    for did in range(1, n + 1):
        mics_nodes.append(Node(
            package="mics_nodes", executable="aircraft",
            name=f"aircraft_{did}", output="screen",
            parameters=params + [{"drone_id": did}]))

    # Phase 2: runtime RTF is a live knob in Gazebo (unlike software-sim, whose
    # tick rate is fixed at launch). The world ships a default real_time_factor,
    # but we override it once the server is up by calling the physics service,
    # so `rtf:=` takes effect without editing the SDF. This is the same lever a
    # future orchestrator would pull to change speed mid-run.
    set_rtf = ExecuteProcess(
        cmd=["ign", "service", "-s", "/world/mics/set_physics",
             "--reqtype", "ignition.msgs.Physics",
             "--reptype", "ignition.msgs.Boolean",
             "--timeout", "3000",
             "--req", f"real_time_factor: {rtf}"],
        output="screen")

    # Stagger: server first, then spawn entities + bridge once it is up, then
    # the MICS graph. Entities must exist before nodes command/observe them.
    return [
        gz_server,
        bridge,
        TimerAction(period=3.0, actions=spawns + [set_rtf]),
        TimerAction(period=5.0, actions=[clock_bridge] + mics_nodes),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("scenario", default_value="/scenarios/happy_path.yaml"),
        DeclareLaunchArgument(
            "gz_dir", default_value="/gz",
            description="Directory holding worlds/ and models/ for Gazebo."),
        DeclareLaunchArgument(
            "rtf", default_value="2.0",
            description="Real-time factor applied to the Gazebo world at boot."),
        OpaqueFunction(function=_spawn),
    ])
