#!/bin/bash
# Reconnects to the running colmag ROS container.
# Usage: bash ros/docker_connect.sh

CONTAINER_NAME="colmag_ros"

if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "Container '$CONTAINER_NAME' is not running. Starting it..."
    docker start "$CONTAINER_NAME"
fi

echo "Connecting to $CONTAINER_NAME..."
docker exec -it "$CONTAINER_NAME" bash
