
## Start the localization node
```
ros2 launch turtlebot4_navigation localization.launch.py namespace:=/Turtlebot_02490  map:=/home/gssi-lab/navigation/lab_corridor.yaml
```

## Launch the navigation node
ros2 launch turtlebot4_navigation nav2.launch.py namespace:=/Turtlebot_02490


## RViz
ros2 launch turtlebot4_viz view_robot.launch.py namespace:=/Turtlebot_02490