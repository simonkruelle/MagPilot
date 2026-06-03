FROM ros:noetic-robot

# System deps. python3-tk and the small X/OpenGL libraries allow the full
# Matplotlib UI to run when the container has a forwarded display.
ENV DEBIAN_FRONTEND=noninteractive \
    PIP_NO_CACHE_DIR=1 \
    MPLCONFIGDIR=/tmp/colmag-matplotlib-cache

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    libgl1 \
    libglib2.0-0 \
    libsm6 \
    libxext6 \
    libxrender1 \
    python3-dev \
    python3-pip \
    python3-tk \
    python3-rospy \
    python3-std-msgs \
    python3-geometry-msgs \
    ros-noetic-std-msgs \
    ros-noetic-geometry-msgs \
    ros-noetic-sensor-msgs \
    ros-noetic-rosbridge-suite \
    && rm -rf /var/lib/apt/lists/*

# Python deps for the COLMAG GUI, classifier, serial reader, and ROS bridge.
# Docker builds from the project root so requirements.txt can be copied here.
COPY requirements.txt /tmp/colmag-requirements.txt
RUN python3 -m pip install --upgrade pip setuptools wheel \
    && python3 -m pip install -r /tmp/colmag-requirements.txt

# Create workspace
RUN mkdir -p /catkin_ws/src
WORKDIR /catkin_ws

# Source ROS on every shell
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc
RUN echo "[ -f /catkin_ws/devel/setup.bash ] && source /catkin_ws/devel/setup.bash" >> /root/.bashrc

CMD ["/bin/bash"]
