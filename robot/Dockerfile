FROM osrf/ros:noetic-desktop-full

# Install dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    build-essential \
    git \
    wget \
    xvfb \
    x11-utils \
    mesa-utils \
    && rm -rf /var/lib/apt/lists/*

# Create catkin workspace
RUN mkdir -p /catkin_ws/src
WORKDIR /catkin_ws

# Initialize rosdep (if not already done)
RUN rosdep update || true

# Install TurtleBot3 packages
RUN apt-get update && apt-get install -y \
    ros-noetic-turtlebot3 \
    ros-noetic-turtlebot3-simulations \
    ros-noetic-turtlebot3-gazebo \
    ros-noetic-rosbridge-server \
    ros-noetic-web-video-server \
    && rm -rf /var/lib/apt/lists/*

# Set up environment
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc
RUN echo "export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:/opt/ros/noetic/share/turtlebot3_gazebo/models" >> ~/.bashrc

# Copy launch file
COPY sim_stack.launch /catkin_ws/sim_stack.launch

# Build catkin workspace
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# Expose ports
EXPOSE 9090 8080

# Set environment variables for headless operation with proper OpenGL
ENV DISPLAY=:99
ENV LIBGL_ALWAYS_SOFTWARE=1
ENV QT_X11_NO_MITSHM=1
ENV OGRE_RTT_MODE=Copy
ENV GAZEBO_MODEL_PATH=/opt/ros/noetic/share/turtlebot3_gazebo/models:$GAZEBO_MODEL_PATH

# Create startup script
RUN echo '#!/bin/bash\n\
set -e\n\
\n\
# Start Xvfb\n\
Xvfb :99 -screen 0 1024x768x24 -ac +extension GLX +render -noreset &\n\
export DISPLAY=:99\n\
\n\
# Wait for X server to start\n\
sleep 3\n\
\n\
# Source ROS environment\n\
source /opt/ros/noetic/setup.bash\n\
source /catkin_ws/devel/setup.bash\n\
\n\
# Set TurtleBot3 model\n\
export TURTLEBOT3_MODEL=waffle\n\
\n\
# Start roscore in background\n\
roscore &\n\
ROSCORE_PID=$!\n\
\n\
# Wait for roscore to start\n\
sleep 5\n\
\n\
# Launch the simulation\n\
roslaunch /catkin_ws/sim_stack.launch\n\
' > /start_simulation.sh && chmod +x /start_simulation.sh

CMD ["/start_simulation.sh"]
