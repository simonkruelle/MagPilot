#!/bin/bash
# Creates and starts the colmag ROS Noetic Docker container.
# Run once. Afterwards use docker_connect.sh to reopen it.
#
# Usage: bash ros/docker_setup.sh

set -e

PROJECT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
CONTAINER_NAME="colmag_ros"
IMAGE_NAME="colmag_ros:noetic"
ROS_WS="/catkin_ws"
DOCKER_PLATFORM="${DOCKER_PLATFORM:-}"
# EasyOCR (CPU-only) is on by default so the classifier stack is always baked
# into the image. Build a light ROS-only image with INSTALL_EASYOCR=0.
INSTALL_EASYOCR="${INSTALL_EASYOCR:-${INSTALL_FULL_PYTHON_DEPS:-1}}"
INSTALL_FULL_PYTHON_DEPS="${INSTALL_FULL_PYTHON_DEPS:-0}"
INSTALL_GAZEBO="${INSTALL_GAZEBO:-0}"
INSTALL_MOVEIT="${INSTALL_MOVEIT:-0}"
ENABLE_GPU="${COLMAG_ENABLE_GPU:-auto}"
COLMAG_SERIAL_DEVICE="${COLMAG_SERIAL_DEVICE:-}"

BUILD_PLATFORM_ARGS=()
RUN_PLATFORM_ARGS=()
if [ -n "$DOCKER_PLATFORM" ]; then
    BUILD_PLATFORM_ARGS=(--platform "$DOCKER_PLATFORM")
    RUN_PLATFORM_ARGS=(--platform "$DOCKER_PLATFORM")
fi

SERIAL_DEVICE="$COLMAG_SERIAL_DEVICE"
if [ -z "$SERIAL_DEVICE" ]; then
    for candidate in /dev/ttyUSB* /dev/ttyACM*; do
        if [ -e "$candidate" ]; then
            SERIAL_DEVICE="$candidate"
            break
        fi
    done
fi

DEVICE_ARGS=()
if [ -n "$SERIAL_DEVICE" ]; then
    if [ -e "$SERIAL_DEVICE" ]; then
        DEVICE_ARGS=(--device "$SERIAL_DEVICE:$SERIAL_DEVICE")
    else
        echo "WARNING: serial device '$SERIAL_DEVICE' does not exist; not mounting it."
    fi
fi

DISPLAY_ARGS=()
if [ -n "${DISPLAY:-}" ] && [ -d /tmp/.X11-unix ]; then
    DISPLAY_ARGS=(
        -e "DISPLAY=$DISPLAY"
        -e "QT_X11_NO_MITSHM=1"
        -v "/tmp/.X11-unix:/tmp/.X11-unix:rw"
    )

    XAUTHORITY_FILE="${XAUTHORITY:-$HOME/.Xauthority}"
    if [ -f "$XAUTHORITY_FILE" ]; then
        DISPLAY_ARGS+=(
            -e "XAUTHORITY=/tmp/colmag-docker.xauth"
            -v "$XAUTHORITY_FILE:/tmp/colmag-docker.xauth:ro"
        )
    else
        echo "WARNING: XAUTHORITY file not found; GUI apps may not be allowed to open windows."
    fi
fi

# GPU passthrough — enables hardware-accelerated Gazebo/rviz rendering.
# We only add GPU flags when the mechanism actually exists, otherwise the
# container fails to start (Docker 29's --gpus needs a CDI spec; --runtime=nvidia
# needs the nvidia-container-runtime binary). Without it Gazebo still runs, just
# with slower software (Mesa) rendering.
# Set COLMAG_ENABLE_GPU=0 to force off, or =1 to require it (warns if unavailable).
GPU_ARGS=()
GPU_MODE="none (software rendering)"
if [ "$ENABLE_GPU" != "0" ] && command -v nvidia-smi >/dev/null 2>&1; then
    if docker info 2>/dev/null | grep -qiE 'Runtimes:.*\bnvidia\b' && command -v nvidia-container-runtime >/dev/null 2>&1; then
        # Legacy nvidia-container-runtime path
        GPU_ARGS=(--runtime=nvidia -e "NVIDIA_VISIBLE_DEVICES=all" -e "NVIDIA_DRIVER_CAPABILITIES=all")
        GPU_MODE="NVIDIA (runtime=nvidia)"
    elif ls /etc/cdi/*.yaml /etc/cdi/*.json /var/run/cdi/* >/dev/null 2>&1; then
        # Modern CDI path (Docker 25+/29)
        GPU_ARGS=(--gpus all -e "NVIDIA_VISIBLE_DEVICES=all" -e "NVIDIA_DRIVER_CAPABILITIES=all")
        GPU_MODE="NVIDIA (CDI)"
    fi
fi
if [ "${#GPU_ARGS[@]}" -eq 0 ] && [ "$ENABLE_GPU" = "1" ]; then
    echo "WARNING: COLMAG_ENABLE_GPU=1 but no working NVIDIA Docker integration found."
    echo "         Install it with: sudo apt-get install -y nvidia-container-toolkit && \\"
    echo "           sudo nvidia-ctk runtime configure --runtime=docker && sudo systemctl restart docker"
    echo "         Continuing with software rendering."
fi

echo "=== colmag ROS 1 Docker Setup ==="
echo "Project: $PROJECT_DIR"
echo "Container: $CONTAINER_NAME"
echo "EasyOCR deps: $INSTALL_EASYOCR"
echo "Gazebo + Panda: $INSTALL_GAZEBO"
echo "MoveIt: $INSTALL_MOVEIT"
echo "GPU: $GPU_MODE"
if [ -n "$DOCKER_PLATFORM" ]; then
    echo "Platform: $DOCKER_PLATFORM"
else
    echo "Platform: Docker host default"
fi
if [ -n "$SERIAL_DEVICE" ] && [ -e "$SERIAL_DEVICE" ]; then
    echo "Serial device: $SERIAL_DEVICE"
else
    echo "Serial device: none mounted"
fi
if [ "${#DISPLAY_ARGS[@]}" -gt 0 ]; then
    echo "Display: $DISPLAY"
else
    echo "Display: none forwarded"
fi
echo ""

# Build the image
echo "[1/3] Building Docker image ($IMAGE_NAME)..."
docker build \
    "${BUILD_PLATFORM_ARGS[@]}" \
    --build-arg "INSTALL_EASYOCR=$INSTALL_EASYOCR" \
    --build-arg "INSTALL_FULL_PYTHON_DEPS=$INSTALL_FULL_PYTHON_DEPS" \
    --build-arg "INSTALL_GAZEBO=$INSTALL_GAZEBO" \
    --build-arg "INSTALL_MOVEIT=$INSTALL_MOVEIT" \
    -f "$PROJECT_DIR/ros/Dockerfile" \
    -t "$IMAGE_NAME" \
    "$PROJECT_DIR"

# Validate the GPU runtime actually WORKS before baking it into the container.
# Detecting that the binary exists is not enough: a mismatched/outdated
# nvidia-container-toolkit (e.g. the segfaulting 1.12.x against a newer driver)
# would make the container fail to even start. If the probe fails, fall back to
# software rendering so the container still comes up.
if [ "${#GPU_ARGS[@]}" -gt 0 ]; then
    echo "Validating GPU runtime ($GPU_MODE)..."
    if docker run --rm "${GPU_ARGS[@]}" "$IMAGE_NAME" true >/dev/null 2>&1; then
        echo "GPU runtime OK."
    else
        echo "WARNING: GPU runtime probe failed — the NVIDIA container runtime is present"
        echo "         but not working (commonly an outdated nvidia-container-toolkit vs. driver)."
        echo "         Falling back to software rendering. To enable GPU, upgrade the toolkit:"
        echo "           sudo apt-get install -y \\"
        echo "             nvidia-container-toolkit=1.19.1-1 nvidia-container-toolkit-base=1.19.1-1 \\"
        echo "             libnvidia-container-tools=1.19.1-1 libnvidia-container1=1.19.1-1 && \\"
        echo "           sudo systemctl restart docker"
        GPU_ARGS=()
        GPU_MODE="none (software rendering; GPU probe failed)"
    fi
fi

# Remove existing container if present
if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "Removing existing container '$CONTAINER_NAME'..."
    docker rm -f "$CONTAINER_NAME"
fi

# Create the container — mount project at /colmag inside the container
echo "[2/3] Creating container '$CONTAINER_NAME'..."
docker run -dit \
    --name "$CONTAINER_NAME" \
    "${RUN_PLATFORM_ARGS[@]}" \
    "${DEVICE_ARGS[@]}" \
    "${DISPLAY_ARGS[@]}" \
    "${GPU_ARGS[@]}" \
    --network host \
    -v "$PROJECT_DIR:/colmag" \
    -v "$PROJECT_DIR/ros/colmag_ros:$ROS_WS/src/colmag_ros" \
    -v "colmag_easyocr_cache:/root/.EasyOCR" \
    -v "colmag_torch_cache:/root/.cache/torch" \
    -e "ROS_PACKAGE_PATH=$ROS_WS/src:/opt/ros/noetic/share" \
    "$IMAGE_NAME" \
    /bin/bash

# Build the catkin workspace inside the container
echo "[3/3] Building catkin workspace..."
docker exec "$CONTAINER_NAME" bash -lc "
    set -e
    source /opt/ros/noetic/setup.bash && \
    cd $ROS_WS && \
    catkin_make
"

echo ""
echo "=== Setup complete ==="
echo "Connect:    bash ros/docker_connect.sh"
echo "Run node:   roslaunch colmag_ros colmag.launch"
echo "EasyOCR:    INSTALL_EASYOCR=1 bash ros/docker_setup.sh"
echo "Gazebo:     INSTALL_GAZEBO=1 bash ros/docker_setup.sh"
if [ "$INSTALL_GAZEBO" = "1" ]; then
    echo "Spawn Panda (inside the container, with a forwarded display):"
    echo "  roslaunch franka_gazebo panda.launch interactive_marker:=true \\"
    echo "    controller:=cartesian_impedance_example_controller"
    echo "Headless smoke test (no GUI):"
    echo "  roslaunch franka_gazebo panda.launch headless:=true"
fi
echo ""
echo "To connect to the lab robot's ROS master, inside the container:"
echo "  export ROS_MASTER_URI=http://<lab-robot-ip>:11311"
echo "  export ROS_IP=\$(hostname -I | awk '{print \$1}')"
