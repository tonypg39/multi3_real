* Allow for Docker GUI
`xhost +local:docker`

* Copy the launch file into the Humble tb4 navigation dir
`cp bringup_nav.launch.py /opt/ros/humble/turtlebot4_navigation/launch`


* Manual run of the commands
```
ros2 launch turtlebot4_navigation localization.launch.py map:=/home/ubuntu/navigation/lab_empty.yaml namespace:=/Turtlebot_02489
ros2 launch turtlebot4_navigation nav2.launch.py namespace:=/Turtlebot_02493
ros2 launch turtlebot4_viz view_robot.launch.py namespace:=/Turtlebot_02493
```