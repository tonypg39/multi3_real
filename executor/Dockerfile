FROM ros:humble

# Environment
ENV DEBIAN_FRONTEND=noninteractive

# Create workspace
WORKDIR /multi3_exec_ws

# Copy the ROS2 workspace into the container
COPY multi3_exec_ws/src ./src

# Install ROS 2 dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    build-essential \
    ros-humble-nav2-msgs \ 
    ros-humble-rviz2 \
    ros-humble-turtlebot4-navigation \
    ros-humble-turtlebot4-viz \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages if needed
RUN pip3 install requests flask

# Copy the combined navigation script
COPY setup/bringup_nav.launch.py /opt/ros/humble/share/turtlebot4_navigation/launch

COPY setup/navigation .

# Source ROS and build the workspace
RUN . /opt/ros/humble/setup.sh && \
    colcon build

# Setup entrypoint
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
