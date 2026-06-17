#!/usr/bin/env python3
"""
fr3_simple_move.py — minimal real/sim Franka trajectory smoke test.

The node waits for the current joint state, nudges one joint by a small delta,
then returns to the starting pose. It is meant to verify that franka_control,
controller_manager and FollowJointTrajectory are wired correctly before running
the richer COLMAG gesture motions.
"""

import sys

import actionlib
import rospy
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


class SimpleFr3Move:
    def __init__(self):
        rospy.init_node("fr3_simple_move", anonymous=False)

        self.arm_id = rospy.get_param("~arm_id", "fr3")
        self.controller = rospy.get_param("~arm_controller", "position_joint_trajectory_controller")
        self.joint_index = int(rospy.get_param("~joint_index", 7))
        self.delta = float(rospy.get_param("~delta", 0.08))
        self.dry_run = bool(rospy.get_param("~dry_run", True))
        self.start_delay = float(rospy.get_param("~start_delay", 1.0))
        self.move_time = float(rospy.get_param("~move_time", 3.0))
        self.return_time = float(rospy.get_param("~return_time", 6.0))
        self.joint_state_topic = rospy.get_param(
            "~joint_state_topic", "/franka_state_controller/joint_states"
        )

        if not 1 <= self.joint_index <= 7:
            raise ValueError("~joint_index must be in 1..7")

        self.joint_names = ["{}_joint{}".format(self.arm_id, i) for i in range(1, 8)]

    def _current_positions(self):
        rospy.loginfo("Waiting for joint state on %s ...", self.joint_state_topic)
        msg = rospy.wait_for_message(self.joint_state_topic, JointState, timeout=10.0)
        positions_by_name = dict(zip(msg.name, msg.position))
        missing = [name for name in self.joint_names if name not in positions_by_name]
        if missing:
            raise RuntimeError(
                "Joint state is missing expected joints for arm_id='{}': {}".format(
                    self.arm_id, ", ".join(missing)
                )
            )
        return [positions_by_name[name] for name in self.joint_names]

    def _trajectory_goal(self, start):
        target = list(start)
        target[self.joint_index - 1] += self.delta

        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = list(self.joint_names)

        start_point = JointTrajectoryPoint()
        start_point.positions = list(start)
        start_point.time_from_start = rospy.Duration(self.start_delay)

        target_point = JointTrajectoryPoint()
        target_point.positions = target
        target_point.time_from_start = rospy.Duration(self.move_time)

        return_point = JointTrajectoryPoint()
        return_point.positions = list(start)
        return_point.time_from_start = rospy.Duration(self.return_time)

        goal.trajectory.points = [start_point, target_point, return_point]
        return goal, target

    def run(self):
        start = self._current_positions()
        goal, target = self._trajectory_goal(start)

        rospy.loginfo("Start joints:  %s", ["{:.3f}".format(v) for v in start])
        rospy.loginfo("Target joints: %s", ["{:.3f}".format(v) for v in target])

        action_ns = "/{}/follow_joint_trajectory".format(self.controller)
        if self.dry_run:
            rospy.logwarn("Dry run only. Set _dry_run:=false to send to %s", action_ns)
            return

        rospy.loginfo("Connecting to %s ...", action_ns)
        client = actionlib.SimpleActionClient(action_ns, FollowJointTrajectoryAction)
        if not client.wait_for_server(rospy.Duration(10.0)):
            raise RuntimeError("Trajectory action server not available: {}".format(action_ns))

        rospy.logwarn("Sending small real-robot trajectory in 2 seconds. Keep the E-stop reachable.")
        rospy.sleep(2.0)
        goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.2)
        client.send_goal(goal)
        client.wait_for_result(rospy.Duration(self.return_time + 3.0))

        result = client.get_result()
        state = client.get_state()
        rospy.loginfo("Trajectory finished with action state %s, result=%s", state, result)


def main():
    try:
        SimpleFr3Move().run()
    except Exception as exc:
        rospy.logerr("%s", exc)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
