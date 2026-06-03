FROM ros:noetic-robot

ARG INSTALL_EASYOCR=1
ARG INSTALL_FULL_PYTHON_DEPS=0
ARG INSTALL_GAZEBO=0
ARG INSTALL_MOVEIT=0

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
    python3-argcomplete \
    python3-dev \
    python3-matplotlib \
    python3-numpy \
    python3-pil \
    python3-pip \
    python3-serial \
    python3-tk \
    python3-rospy \
    python3-std-msgs \
    python3-geometry-msgs \
    ros-noetic-std-msgs \
    ros-noetic-geometry-msgs \
    ros-noetic-sensor-msgs \
    ros-noetic-rosbridge-suite \
    && rm -rf /var/lib/apt/lists/*

# The CPU-only EasyOCR/PyTorch stack is installed by default and baked into the
# image, so it survives container recreation by docker_setup.sh. It uses the
# CPU wheel index pinned in requirements-docker-ocr.txt (no multi-GB CUDA
# download). For a light ROS-only image, build with INSTALL_EASYOCR=0.
COPY requirements-docker-ocr.txt /tmp/colmag-ocr-requirements.txt
RUN if [ "$INSTALL_EASYOCR" = "1" ] || [ "$INSTALL_FULL_PYTHON_DEPS" = "1" ]; then \
        python3 -m pip install --upgrade "pip<25" setuptools wheel && \
        python3 -m pip install -r /tmp/colmag-ocr-requirements.txt; \
    else \
        echo "Skipping EasyOCR (INSTALL_EASYOCR=0)."; \
    fi

# Optional: Gazebo 11 + Franka Emika Panda simulation stack.
# ros:noetic-robot does not ship a simulator, so this is opt-in to keep the
# default image small. INSTALL_GAZEBO=1 adds Gazebo, gazebo_ros_control (needed
# by franka_gazebo, otherwise you get "Failed to create robot simulation
# interface loader"), libfranka and the franka_ros stack — which vendors the
# Panda URDF/meshes and franka_gazebo's panda.launch.
RUN if [ "$INSTALL_GAZEBO" = "1" ]; then \
        apt-get update && apt-get install -y --no-install-recommends \
            ros-noetic-gazebo-ros-pkgs \
            ros-noetic-gazebo-ros-control \
            ros-noetic-libfranka \
            ros-noetic-franka-ros && \
        rm -rf /var/lib/apt/lists/*; \
    else \
        echo "Skipping Gazebo. Set INSTALL_GAZEBO=1 to add Gazebo + Panda."; \
    fi

# Optional: MoveIt + panda_moveit_config, so colmag_robot_node.py can send
# motion-planning goals to the simulated arm instead of raw joint commands.
# Adds a few hundred MB on top of Gazebo.
RUN if [ "$INSTALL_MOVEIT" = "1" ]; then \
        apt-get update && apt-get install -y --no-install-recommends \
            ros-noetic-moveit \
            ros-noetic-panda-moveit-config && \
        rm -rf /var/lib/apt/lists/*; \
    else \
        echo "Skipping MoveIt. Set INSTALL_MOVEIT=1 to add it."; \
    fi

# Create workspace
RUN mkdir -p /catkin_ws/src
WORKDIR /catkin_ws

# Source ROS on every shell
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc
RUN echo "[ -f /catkin_ws/devel/setup.bash ] && source /catkin_ws/devel/setup.bash" >> /root/.bashrc

CMD ["/bin/bash"]
