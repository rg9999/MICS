# MICS — headless ROS2 Humble runtime (PRD §4.5 CPU path).
#
# No Gazebo/PX4 SITL here (those need GPU + more RAM than this host exposes).
# The drone/attacker physics are the lightweight kinematic models from the
# mics core, so the whole multi-vehicle scenario runs on CPU in one container.
FROM ros:humble-ros-base

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-numpy \
        python3-yaml \
        python3-pip \
        python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/mics

# Behaviour-defining core, installed into the system interpreter that the ROS2
# Python nodes use (no venv — ament_python entry points run under /usr/bin).
COPY pyproject.toml requirements.txt ./
COPY mics ./mics
# Stock humble setuptools predates PEP 621 (builds an empty "UNKNOWN" wheel),
# but setuptools>=80 drops `setup.py develop` which colcon --symlink-install
# needs for ament_python. Pin into the window that supports both.
RUN pip install --no-cache-dir --upgrade pip "setuptools>=61,<80" wheel \
    && pip install --no-cache-dir .

# ROS2 interfaces + nodes
COPY ros2_ws/src ./ros2_ws/src
RUN source /opt/ros/humble/setup.bash \
    && cd ros2_ws \
    && colcon build --symlink-install

# viewer-gateway (ROS<->browser choke point). Runs from /opt/mics via
# `python3 -m viewer_gateway`; only its `ros` source imports rclpy/mics_msgs.
RUN pip install --no-cache-dir "websockets>=12"
COPY viewer_gateway ./viewer_gateway

COPY scenarios /scenarios
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "mics_nodes", "mics.launch.py", "scenario:=/scenarios/happy_path.yaml"]
