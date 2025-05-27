* Allow for Docker GUI
`xhost +local:docker`

* Copy the launch file into the Humble tb4 navigation dir
`cp bringup_nav.launch.py /opt/ros/humble/turtlebot4_navigation/launch`


* Manual run of the commands
```
<<<<<<< HEAD
ros2 launch turtlebot4_navigation localization.launch.py map:=/home/gssi-lab/testmap.yaml namespace:=/Turtlebot_02489
ros2 launch turtlebot4_navigation nav2.launch.py namespace:=/Turtlebot_02489
=======
ros2 launch turtlebot4_navigation localization.launch.py map:=/home/ubuntu/navigation/lab_empty.yaml namespace:=/Turtlebot_02489
ros2 launch turtlebot4_navigation nav2.launch.py namespace:=/Turtlebot_02493
>>>>>>> 6de6da0cd4a3ff337ca91aeb456bd00d5ac1f855
ros2 launch turtlebot4_viz view_robot.launch.py namespace:=/Turtlebot_02493
```


docker exec coordinator 