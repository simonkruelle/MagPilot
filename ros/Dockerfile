FROM ros:noetic-robot

# System deps
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-rospy \
    python3-std-msgs \
    python3-geometry-msgs \
    ros-noetic-std-msgs \
    ros-noetic-geometry-msgs \
    ros-noetic-sensor-msgs \
    && rm -rf /var/lib/apt/lists/*

# Python deps for the colmag project (no matplotlib/GUI in the node itself)
RUN pip3 install numpy pyserial

# Create workspace
RUN mkdir -p /catkin_ws/src
WORKDIR /catkin_ws

# Source ROS on every shell
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc
RUN echo "[ -f /catkin_ws/devel/setup.bash ] && source /catkin_ws/devel/setup.bash" >> /root/.bashrc

CMD ["/bin/bash"]
