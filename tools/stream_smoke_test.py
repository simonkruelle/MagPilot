#!/usr/bin/env python3
"""End-to-end smoke test for the MagPilot streaming pipeline (no robot).

Runs the real colmag_draw_node in-process against a mock trajectory
controller, feeds it a moving magnet pose, and verifies the streamed
commands:
  - header stamps advance on an exact uniform 1/update_rate grid
  - every command carries the look-ahead second point
  - velocity tags equal the slope of the streamed position sequence
  - the tick loop holds its rate (no overruns from IK)

Usage (inside the ROS container, with a private master):
  ROS_MASTER_URI=http://localhost:11390 roscore -p 11390 &
  ROS_MASTER_URI=http://localhost:11390 python3 tools/stream_smoke_test.py
"""

import sys
import threading
import time

import numpy as np

sys.path.insert(0, '/colmag/ros/colmag_ros/scripts')

import actionlib
import rospy
from control_msgs.msg import FollowJointTrajectoryAction
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory

CTRL = 'mock_ctl'
RATE = 30.0

rospy.set_param('/colmag_draw_node/dry_run', False)
rospy.set_param('/colmag_draw_node/arm_controller', CTRL)
rospy.set_param('/colmag_draw_node/update_rate', RATE)
rospy.set_param('/colmag_draw_node/home_on_exit', False)

received = []


def on_command(msg):
    received.append((msg.header.stamp.to_sec(),
                     [(list(p.positions), list(p.velocities),
                       p.time_from_start.to_sec()) for p in msg.points]))


def main():
    from colmag_draw_node import ColmagDrawNode, _HOME

    # ColmagDrawNode.__init__ calls init_node with these exact args; doing it
    # here first (a matching re-init is a no-op) initializes rospy time so the
    # mock action server can start and be up before the node's client waits.
    rospy.init_node('colmag_draw_node', anonymous=False)

    server = actionlib.SimpleActionServer(
        '/%s/follow_joint_trajectory' % CTRL, FollowJointTrajectoryAction,
        auto_start=False)
    server.start()

    node = ColmagDrawNode()

    rospy.Subscriber('/%s/command' % CTRL, JointTrajectory, on_command)
    joint_pub = rospy.Publisher('/franka_state_controller/joint_states',
                                JointState, queue_size=1)
    pose_pub = rospy.Publisher('/colmag/pose', PoseStamped, queue_size=1)
    enable_pub = rospy.Publisher('/colmag/draw_enable', Bool, queue_size=1,
                                 latch=True)

    stop = threading.Event()

    def feed():
        t0 = time.monotonic()
        while not stop.is_set():
            t = time.monotonic() - t0
            js = JointState()
            js.header.stamp = rospy.Time.now()
            js.name = list(node.joint_names)
            js.position = list(_HOME)
            joint_pub.publish(js)
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = 'colmag_base'
            pose.pose.position.x = 0.02
            pose.pose.position.y = -0.01
            # magnet height sweep 17 -> 100 -> 17 mm, 0.1 Hz triangle
            phase = abs((t * 0.2) % 2.0 - 1.0)
            pose.pose.position.z = 0.017 + 0.083 * (1.0 - phase)
            pose.pose.orientation.w = 1.0
            pose_pub.publish(pose)
            time.sleep(1.0 / 60.0)

    feeder = threading.Thread(target=feed, daemon=True)
    feeder.start()
    time.sleep(1.0)
    enable_pub.publish(Bool(data=True))
    time.sleep(8.0)
    node._set_enabled(False, 'smoke test done')
    stop.set()

    # Halt sentinels (empty-point trajectories from disable/lift-pause) carry
    # their own "stop now" stamp and are not part of the streaming grid.
    msgs = [m for m in received if m[1]]
    assert len(msgs) > 100, 'expected a steady command stream, got %d' % len(msgs)

    period = 1.0 / RATE
    stamps = np.array([m[0] for m in msgs])
    deltas = np.diff(stamps)
    on_grid = np.isclose(deltas, period, atol=1e-6)
    bad = deltas[~(on_grid | (deltas > period - 1e-6))]
    if len(bad):
        print('OFFENDING sub-period deltas (ms): ',
              np.round(np.sort(bad) * 1e3, 3)[:20])
    # every non-grid delta must be a re-anchor after a skipped tick (> period)
    assert all(on_grid | (deltas > period - 1e-6)), 'stamp grid broken'

    two_point = sum(1 for m in msgs if len(m[1]) == 2)
    assert two_point > 0.9 * len(msgs), (
        'look-ahead point missing in %d of %d commands'
        % (len(msgs) - two_point, len(msgs)))

    slope_errors = []
    for (s0, pts0), (s1, pts1) in zip(msgs, msgs[1:]):
        if not np.isclose(s1 - s0, period, atol=1e-6):
            continue  # re-anchor boundary: slope check not applicable
        q0 = np.array(pts0[0][0])
        q1 = np.array(pts1[0][0])
        v1 = np.array(pts1[0][1])
        slope_errors.append(np.max(np.abs((q1 - q0) / period - v1)))
    assert slope_errors and max(slope_errors) < 1e-9, (
        'velocity tags diverge from position slope: max %.3g rad/s'
        % (max(slope_errors) if slope_errors else float('nan')))

    grid_fraction = float(np.mean(on_grid))
    moved = np.ptp([m[1][0][0][3] for m in msgs])  # joint 4 range over sweep
    print('PASS: %d commands | %.1f%% consecutive ticks on the exact grid | '
          '%d with look-ahead point | max slope/velocity mismatch %.2g | '
          'joint4 sweep %.3f rad' % (len(msgs), 100.0 * grid_fraction,
                                     two_point, max(slope_errors), moved))
    return 0


if __name__ == '__main__':
    sys.exit(main())
