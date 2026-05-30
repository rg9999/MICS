#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
source /opt/mics/ros2_ws/install/setup.bash
exec "$@"
