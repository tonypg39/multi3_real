* Allow for Docker GUI
`xhost +local:docker`

* Copy the launch file into the Humble tb4 navigation dir
`cp bringup_nav.launch.py /opt/ros/humble/turtlebot4_navigation/launch`


* Manual run of the commands
```
ros2 launch turtlebot4_navigation localization.launch.py map:=/home/gssi-lab/multi3_repos/multi3_real/executor/setup/navigation/lab_empty.yaml namespace:=/Turtlebot_02490
ros2 launch turtlebot4_navigation nav2.launch.py namespace:=/Turtlebot_02490
ros2 launch turtlebot4_viz view_robot.launch.py namespace:=/Turtlebot_02490
```


docker exec coordinator 