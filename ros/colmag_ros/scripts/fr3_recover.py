#!/usr/bin/env python3
"""fr3_recover.py — clear the Franka error state after an E-stop or reflex stop.

When the external activation device (E-stop) is pressed, or a safety reflex
trips (velocity/force limit, collision), the FR3 locks itself and
franka_control refuses further commands until an error-recovery request is
sent. This script sends that request via the action franka_control exposes.

Usage (with fr3_real.launch running):
  rosrun colmag_ros fr3_recover.py

Order of operations after an emergency stop:
  1. Release the activation device (twist/pull it back out).
  2. Run this script.
  3. If the arm is still locked: unlock the joints in the Desk web interface
     (brakes) and check that FCI mode is active, then run this again.
  4. If fr3_real.launch itself died (franka_control is a required process),
     simply restart it — a fresh connection also clears the error state.
"""

import sys

import rospy


def main():
    try:
        import actionlib
        from franka_msgs.msg import ErrorRecoveryAction, ErrorRecoveryGoal
    except ImportError as exc:
        print("franka_msgs/actionlib not importable ({}). Run this inside the "
              "Franka Docker image (INSTALL_GAZEBO=1 build).".format(exc))
        return 1

    rospy.init_node("fr3_recover", anonymous=True)
    action_ns = rospy.get_param("~action_ns", "/franka_control/error_recovery")

    client = actionlib.SimpleActionClient(action_ns, ErrorRecoveryAction)
    rospy.loginfo("Waiting for %s ...", action_ns)
    if not client.wait_for_server(rospy.Duration(5.0)):
        rospy.logerr("Error-recovery server not available. Is fr3_real.launch "
                     "running? If franka_control died, restart the launch instead.")
        return 1

    rospy.loginfo("Sending error recovery request ...")
    client.send_goal(ErrorRecoveryGoal())
    if not client.wait_for_result(rospy.Duration(10.0)):
        rospy.logwarn("No result within 10 s. If the arm stays locked: release the "
                      "activation device, unlock the joints in Desk, then retry.")
        return 1

    rospy.loginfo("Recovery request done (action state %s). The arm should accept "
                  "trajectories again.", client.get_state())
    return 0


if __name__ == "__main__":
    sys.exit(main())
