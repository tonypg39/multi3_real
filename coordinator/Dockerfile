FROM ros:humble

WORKDIR /multi3_coord_ws

COPY multi3_coord_ws/src ./src
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

RUN apt update && apt install -y python3-pip && \
    pip3 install flask

RUN . /opt/ros/humble/setup.sh && colcon build

ENTRYPOINT ["/entrypoint.sh"]
