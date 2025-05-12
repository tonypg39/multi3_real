#!/bin/bash
source /opt/ros/humble/setup.bash
cd /multi3_coord_ws
colcon build
source install/setup.bash
exec ros2 run multi3_coordinator coordinator
