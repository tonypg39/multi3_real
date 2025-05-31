#!/bin/bash
set -e

# Source ROS 2 and workspace
source /opt/ros/humble/setup.bash
source /multi3_exec_ws/install/setup.bash

# Launch the node
echo "Starting the navigation stack..."
# exec ros2 topic list

# exec ros2 launch turtlebot4_navigation bringup_nav.launch.py namespace:=/$TB_ID map:=/opt/ros/humble/share/turtlebot4_navigation/navigation/testmap.yaml &
# exec ros2 launch turtlebot4_viz view_robot.launch.py namespace:=/$TB_ID &

echo $TB_ID
# sleep infinity # wait for the Navigation stack to load up

exec ros2 run multi3_executor executor
