#!/bin/bash
# docker_reset.sh — return the colmag_simon container to a pristine state.
#
# Kills EVERY ROS process inside it (the ROS master/roscore, gazebo,
# franka_control, the colmag arm nodes and the interface) and removes the
# legacy host-network container that would otherwise leak nodes onto the shared
# localhost:11311 master. It does NOT rebuild the image or touch your code —
# only running processes are cleared.
#
# Use it whenever the pipeline is in a half-started or confusing state and you
# want a guaranteed blank slate before starting Robot -> Arm nodes -> Interface.
#
#   bash ros/docker_reset.sh          # full restart (nuclear, always clean)
#   SOFT=1 bash ros/docker_reset.sh   # just pkill everything, keep the container
#
# NOTE: this does NOT clear a Franka safety reflex/E-stop. If the FR3 latched an
# error ("Motion finished commanded, but the robot is still moving!"), you must
# also release the enabling device, unlock the joints in Franka Desk, and run
# error recovery (restart the Robot stage, or `rosrun colmag_ros fr3_recover.py`).

set -e
CONTAINER="colmag_simon"
LEGACY="colmag_ros"
SOFT="${SOFT:-0}"

# 1) Remove the legacy container entirely — with host networking its nodes
#    register on the same master and cannot be killed from colmag_simon.
if docker ps -a --format '{{.Names}}' | grep -qx "$LEGACY"; then
    echo "Removing legacy container '$LEGACY'..."
    docker rm -f "$LEGACY" >/dev/null 2>&1 || true
fi

if ! docker ps --format '{{.Names}}' | grep -qx "$CONTAINER"; then
    echo "Container '$CONTAINER' is not running; starting it clean..."
    docker start "$CONTAINER" >/dev/null
    echo "Done. '$CONTAINER' is clean."
    exit 0
fi

if [ "$SOFT" = "1" ]; then
    # Soft reset: kill the pipeline processes but keep the container up. Killing
    # roslaunch + the master drops every node registration, so no phantom node
    # (e.g. a stale /gazebo) can survive. TERM first, then KILL what remains.
    echo "Soft reset: killing pipeline processes inside '$CONTAINER'..."
    docker exec "$CONTAINER" bash -lc '
        for sig in TERM KILL; do
            pkill -"$sig" -f roslaunch 2>/dev/null || true
            pkill -"$sig" -f magnetometer_reader.py 2>/dev/null || true
            pkill -"$sig" -f colmag_draw_node.py 2>/dev/null || true
            pkill -"$sig" -f colmag_robot_node.py 2>/dev/null || true
            pkill -"$sig" -f franka_control 2>/dev/null || true
            pkill -"$sig" -f franka_gripper 2>/dev/null || true
            pkill -"$sig" -x gzserver 2>/dev/null || true
            pkill -"$sig" -x gzclient 2>/dev/null || true
            pkill -"$sig" -f rosmaster 2>/dev/null || true
            pkill -"$sig" -f roscore 2>/dev/null || true
            sleep 1
        done
        rm -f /tmp/colmag_gui_*.pid /tmp/colmag_gui_*.log 2>/dev/null || true
        true'
    echo "Done. '$CONTAINER' pipeline is clear (container still up)."
else
    # Hard reset: restart the container. This tears down its PID namespace, so
    # the ROS master and every pipeline process are guaranteed gone.
    echo "Restarting '$CONTAINER' for a guaranteed clean slate..."
    docker restart "$CONTAINER" >/dev/null
    docker exec "$CONTAINER" bash -lc 'rm -f /tmp/colmag_gui_*.pid /tmp/colmag_gui_*.log 2>/dev/null || true'
    echo "Done. '$CONTAINER' is clean."
fi

echo ""
echo "Now start fresh (in the Control Center or manually):"
echo "  1. Robot        (Real robot: fr3_real.launch)"
echo "  2. Arm nodes    (LIVE toggle ON so dry_run:=false)"
echo "  3. Interface    (trackpad or magnetometer)"
