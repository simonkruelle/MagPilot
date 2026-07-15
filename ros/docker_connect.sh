#!/bin/bash
# Reconnects to the running colmag ROS container.
# Usage: bash ros/docker_connect.sh

CONTAINER_NAME="colmag_simon"

if ! docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "Container '$CONTAINER_NAME' does not exist. Create it first:"
    echo "  bash ros/docker_setup.sh"
    exit 1
fi

if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "Container '$CONTAINER_NAME' is not running. Starting it..."
    docker start "$CONTAINER_NAME"
fi

echo "Connecting to $CONTAINER_NAME..."
docker exec -it -w /colmag "$CONTAINER_NAME" bash
