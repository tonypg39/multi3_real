#!/bin/bash
set -e

# Source ROS 2 and workspace
source /opt/ros/humble/setup.bash
source /multi3_exec_ws/install/setup.bash

# Launch the node
exec ros2 run multi3_executor executor
