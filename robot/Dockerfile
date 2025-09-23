FROM osrf/ros:noetic-desktop-full

# Paket ROS yang dibutuhin (tanpa Gazebo)
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-rosbridge-server \
    ros-noetic-rosapi \
    ros-noetic-web-video-server \
    ros-noetic-turtlebot3 \
    ros-noetic-turtlebot3-simulations \
    ros-noetic-image-publisher \
    netcat curl \
  && rm -rf /var/lib/apt/lists/*

# Env penting
ENV TURTLEBOT3_MODEL=waffle \
    ROS_IP=127.0.0.1 \
    ROS_MASTER_URI=http://localhost:11311

WORKDIR /root/turtlebot3_ws

# Entrypoint baru yang headless-safe
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

EXPOSE 9090 8080
ENTRYPOINT ["/entrypoint.sh"]
