#!/usr/bin/env python3
"""
colmag_robot_node.py — robot action executor for the COLMAG gesture interface.

This is the terminal node of the pipeline.  It receives confirmed commands
from the joystick node and executes the corresponding robot action.

It ships with a set of PREPROGRAMMED demo motions for a 7-DOF Franka arm
(Panda or FR3) running in Gazebo, so you can see the full pipeline move a robot
end-to-end: classify an 'A' and the arm waves; classify a '0' and it returns
home; 'L'/'R' rotate the base, etc.  Extend or replace NAMED_POSES and
execute_command() with your own actions (MoveIt goals, real-robot API, …).

How motion is sent:
  Joint-space FollowJointTrajectory goals are sent to a
  JointTrajectoryController (default 'effort_joint_trajectory_controller', which
  franka_gazebo defines in sim_controllers.yaml).  Launch the arm so that
  controller is running, e.g.:
    roslaunch colmag_ros fr3.launch   headless:=true controller:=effort_joint_trajectory_controller
    roslaunch franka_gazebo panda.launch headless:=true controller:=effort_joint_trajectory_controller

Topics subscribed:
  /colmag/command     (std_msgs/String)   — confirmed command from joystick_node
  /colmag/classifier  (std_msgs/String)   — latest predicted label (for logging)
  /colmag/confidence  (std_msgs/Float64)  — latest confidence (for logging)

Command strings you will receive:
  'letter_detection'     — user switched to letter OCR mode
  'number_detection'     — user switched to digit OCR mode
  'symbol_detection'     — user switched to future signs/shape mode
  'canvas:reset'         — canvas was cleared
  'choice:0'             — user confirmed the top classifier candidate
  'choice:1' … 'choice:3' — user confirmed runner-up candidates

Parameters:
  ~robot_ip        (str,  default '')                              — placeholder for a real robot's IP
  ~dry_run         (bool, default true)                            — if true, log commands but do not move
  ~arm_id          (str,  default 'fr3')                           — joint name prefix: '<arm_id>_joint1..7' (use 'panda' for the Panda)
  ~arm_controller  (str,  default 'effort_joint_trajectory_controller') — JointTrajectoryController to drive
  ~move_duration   (float, default 3.0)                           — seconds for a single-pose move

Usage:
  rosrun colmag_ros colmag_robot_node.py
  # Drive the simulated FR3:
  rosrun colmag_ros colmag_robot_node.py _dry_run:=false _arm_id:=fr3
  # Drive the simulated Panda:
  rosrun colmag_ros colmag_robot_node.py _dry_run:=false _arm_id:=panda
"""

import rospy
from std_msgs.msg import String, Float64

# Actionlib + trajectory messages are only needed to actually drive an arm.
# Import lazily so the node still runs in dry-run on a light image that has no
# controller stack installed.
try:
    import actionlib
    from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
    from trajectory_msgs.msg import JointTrajectoryPoint
    _TRAJ_AVAILABLE = True
except ImportError:
    _TRAJ_AVAILABLE = False


# Joint-space poses for the 7-DOF Franka arm (Panda and FR3 share the joint
# layout: joint1..joint7 in radians).  HOME matches the launch file's default
# elbow-bend pose.  Values are kept conservative so they are valid on both arms.
HOME_POSE = [0.0, -0.785398163, 0.0, -2.35619449, 0.0, 1.57079632679, 0.785398163397]

NAMED_POSES = {
    '0': HOME_POSE,                                          # reset / home
    'X': HOME_POSE,                                          # cancel → home
    'B': [0.00,  0.35, 0.00, -2.10, 0.00, 2.45, 0.785],      # bow forward
    'C': [0.00, -0.30, 0.00, -1.30, 0.00, 1.05, 0.785],      # curl in
    'L': [0.70, -0.785, 0.00, -2.356, 0.00, 1.571, 0.785],   # rotate base left
    'R': [-0.70, -0.785, 0.00, -2.356, 0.00, 1.571, 0.785],  # rotate base right
    'U': [0.00, -1.20, 0.00, -1.70, 0.00, 1.40, 0.785],      # reach up
    'D': [0.00, -0.30, 0.00, -2.70, 0.00, 1.90, 0.785],      # reach down
    '1': [0.00, -0.60, 0.00, -2.20, 0.00, 1.60, 0.785],      # counting pose 1
    '2': [0.00, -0.40, 0.00, -2.00, 0.00, 1.60, 0.785],      # counting pose 2
    '3': [0.00, -0.20, 0.00, -1.80, 0.00, 1.60, 0.785],      # counting pose 3
}
# 'A' is handled specially below as a waving motion (multi-point trajectory).


class ColmagRobotNode:
    def __init__(self):
        rospy.init_node('colmag_robot_node', anonymous=False)

        # ── Parameters ────────────────────────────────────────────────────────
        self.robot_ip      = rospy.get_param('~robot_ip', '')
        self.dry_run       = rospy.get_param('~dry_run',  True)
        self.arm_id        = rospy.get_param('~arm_id', 'fr3')
        self.controller    = rospy.get_param('~arm_controller', 'effort_joint_trajectory_controller')
        self.move_duration = float(rospy.get_param('~move_duration', 3.0))

        self.joint_names = ['{}_joint{}'.format(self.arm_id, i) for i in range(1, 8)]

        # ── Classifier state ──────────────────────────────────────────────────
        self.last_label      = None
        self.last_confidence = 0.0

        # ── Trajectory action client (only when actually moving a robot) ──────
        self.traj_client = None
        if not self.dry_run:
            self._setup_trajectory_client()

        # ── Subscribers ───────────────────────────────────────────────────────
        rospy.Subscriber('/colmag/command',    String,  self.on_command)
        rospy.Subscriber('/colmag/classifier', String,  self.on_classifier)
        rospy.Subscriber('/colmag/confidence', Float64, self.on_confidence)

        mode = ('DRY RUN (no robot actions)' if self.dry_run
                else 'LIVE (arm_id={}, controller={})'.format(self.arm_id, self.controller))
        rospy.loginfo('colmag_robot_node ready | mode={}'.format(mode))

    def _setup_trajectory_client(self):
        if not _TRAJ_AVAILABLE:
            rospy.logwarn('control_msgs/actionlib not importable — cannot send arm '
                          'trajectories. Build the image with INSTALL_GAZEBO=1.')
            return
        action_ns = '/{}/follow_joint_trajectory'.format(self.controller)
        rospy.loginfo('Connecting to trajectory action server %s ...', action_ns)
        client = actionlib.SimpleActionClient(action_ns, FollowJointTrajectoryAction)
        if client.wait_for_server(rospy.Duration(10.0)):
            self.traj_client = client
            rospy.loginfo('Trajectory action server connected.')
        else:
            rospy.logwarn('Trajectory action server %s not available after 10s. '
                          'Is the arm launched with controller:=%s ?',
                          action_ns, self.controller)

    # ── Classifier state tracking ──────────────────────────────────────────────

    def on_classifier(self, msg):
        self.last_label = msg.data

    def on_confidence(self, msg):
        self.last_confidence = msg.data

    # ── Command handling ───────────────────────────────────────────────────────

    def on_command(self, msg):
        cmd = msg.data

        # Resolve 'choice:0' to the actual label for logging
        if cmd == 'choice:0' and self.last_label:
            label = self.last_label
            conf  = self.last_confidence * 100
            rospy.loginfo(
                'CONFIRMED gesture: "%s" (%.1f%%) — %s',
                label, conf, '(dry run)' if self.dry_run else 'executing'
            )
            if not self.dry_run:
                self.execute_command(label)
        elif cmd == 'letter_detection':
            rospy.loginfo('Mode → letters OCR')
        elif cmd == 'number_detection':
            rospy.loginfo('Mode → digits OCR')
        elif cmd == 'symbol_detection':
            rospy.loginfo('Mode → signs / shape-recognition placeholder')
        elif cmd == 'canvas:reset':
            rospy.loginfo('Canvas reset')
        elif cmd.startswith('choice:'):
            idx = cmd.split(':')[1]
            rospy.loginfo('Runner-up choice %s selected', idx)
        else:
            rospy.loginfo('Command received: %s', cmd)

    def execute_command(self, label):
        """
        Execute a preprogrammed robot action for the confirmed character label.

        Extend NAMED_POSES (or replace this method) to map labels to your own
        actions: MoveIt goals, real-robot API calls, gripper commands, etc.

        Args:
            label: The confirmed character, e.g. 'A', '3', 'X'
        """
        if label == 'A':
            rospy.loginfo('Action "A" → wave')
            self._wave()
            return

        pose = NAMED_POSES.get(label)
        if pose is None:
            rospy.logwarn('No preprogrammed action mapped for label "%s" — small nod.', label)
            self._nod()
            return

        rospy.loginfo('Action "%s" → move to joint pose', label)
        self._send_trajectory([(pose, self.move_duration)])

    # ── Motion primitives ───────────────────────────────────────────────────────

    def _send_trajectory(self, points):
        """Send a FollowJointTrajectory goal.

        Args:
            points: list of (positions[7], time_from_start_seconds) tuples.
        Returns:
            True if the goal completed, False otherwise (incl. dry-run/no server).
        """
        # Lazy (re)connect: the arm/controller may have come up after this node
        # started, so don't give up permanently if the initial connect timed out.
        if self.traj_client is None:
            self._setup_trajectory_client()
        if self.traj_client is None:
            rospy.logwarn('No trajectory client — is the arm running with '
                          'controller:=%s ? Skipping motion.', self.controller)
            return False

        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = self.joint_names
        for positions, t in points:
            pt = JointTrajectoryPoint()
            pt.positions     = list(positions)
            pt.velocities    = [0.0] * len(self.joint_names)
            pt.time_from_start = rospy.Duration(t)
            goal.trajectory.points.append(pt)
        goal.trajectory.header.stamp = rospy.Time.now()

        self.traj_client.send_goal(goal)
        finished = self.traj_client.wait_for_result(rospy.Duration(points[-1][1] + 5.0))
        if not finished:
            rospy.logwarn('Trajectory did not finish in time.')
        return finished

    def _wave(self):
        """Raise the arm and wave the wrist (joint7) side to side, then home."""
        ready = [0.0, -0.6, 0.0, -1.6, 0.0, 1.4, 0.785]
        left  = list(ready); left[6]  = 1.6
        right = list(ready); right[6] = -0.2
        self._send_trajectory([
            (ready,     2.0),
            (left,      3.0),
            (right,     4.0),
            (left,      5.0),
            (HOME_POSE, 7.0),
        ])

    def _nod(self):
        """Small wrist nod (joint6) around home — fallback for unmapped labels."""
        up   = list(HOME_POSE); up[5]   = HOME_POSE[5] - 0.25
        down = list(HOME_POSE); down[5] = HOME_POSE[5] + 0.25
        self._send_trajectory([(up, 1.5), (down, 3.0), (HOME_POSE, 4.5)])

    def run(self):
        rospy.spin()


def main():
    node = ColmagRobotNode()
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
