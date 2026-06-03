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

BUILD_PLATFORM_ARGS=()
RUN_PLATFORM_ARGS=()
if [ -n "$DOCKER_PLATFORM" ]; then
    BUILD_PLATFORM_ARGS=(--platform "$DOCKER_PLATFORM")
    RUN_PLATFORM_ARGS=(--platform "$DOCKER_PLATFORM")
fi

echo "=== colmag ROS 1 Docker Setup ==="
echo "Project: $PROJECT_DIR"
echo "Container: $CONTAINER_NAME"
if [ -n "$DOCKER_PLATFORM" ]; then
    echo "Platform: $DOCKER_PLATFORM"
else
    echo "Platform: Docker host default"
fi
echo ""

# Build the image
echo "[1/3] Building Docker image ($IMAGE_NAME)..."
docker build "${BUILD_PLATFORM_ARGS[@]}" -f "$PROJECT_DIR/ros/Dockerfile" -t "$IMAGE_NAME" "$PROJECT_DIR"

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
    --network host \
    -v "$PROJECT_DIR:/colmag" \
    -v "$PROJECT_DIR/ros/colmag_ros:$ROS_WS/src/colmag_ros" \
    -e "ROS_PACKAGE_PATH=$ROS_WS/src:/opt/ros/noetic/share" \
    "$IMAGE_NAME" \
    /bin/bash

# Build the catkin workspace inside the container
echo "[3/3] Building catkin workspace..."
docker exec "$CONTAINER_NAME" bash -c "
    source /opt/ros/noetic/setup.bash && \
    cd $ROS_WS && \
    catkin_make 2>&1 | tail -5
"

echo ""
echo "=== Setup complete ==="
echo "Connect:    bash ros/docker_connect.sh"
echo "Run node:   roslaunch colmag_ros colmag.launch"
echo ""
echo "To connect to the lab robot's ROS master, inside the container:"
echo "  export ROS_MASTER_URI=http://<lab-robot-ip>:11311"
echo "  export ROS_IP=\$(hostname -I | awk '{print \$1}')"
