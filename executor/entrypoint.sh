#!/bin/bash
set -e

# Source ROS 2 and workspace
source /opt/ros/humble/setup.bash
source /multi3_exec_ws/install/setup.bash

# Launch the node
echo "Starting the navigation stack..."
# exec ros2 launch turtlebot4_navigation load_nav.launch.py namespace:=/$TB_ID map:=/home/gssi-lab/navigation/lab_empty.yaml
# ros2 launch turtlebot4_viz view_robot.launch.py namespace:=/$TB_ID

echo $TB_ID
sleep 5 # wait for the Navigation stack to load up

exec ros2 run multi3_executor executor
