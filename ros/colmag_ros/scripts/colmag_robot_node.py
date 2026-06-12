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


# Joint-space home for the 7-DOF Franka arm (Panda and FR3 share the layout:
# joint1..joint7 in radians). HOME matches the launch file's default elbow-bend
# pose. The motion methods below build their waypoints around it; all joint
# values are kept well inside both the Panda and FR3 limits.
HOME_POSE = [0.0, -0.785398163, 0.0, -2.35619449, 0.0, 1.57079632679, 0.785398163397]


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

        # ── Gesture → motion map ──────────────────────────────────────────────
        # Edit this (and the motion methods below) to change the robot's tricks.
        self._motions = {
            '0': self._home,        'X': self._home,         # reset / cancel
            'A': self._wave,        'B': self._bow,          # wave, bow
            'C': self._celebrate,   'D': self._dab,          # fist pumps, dab
            'U': self._stretch_up,                            # reach tall
            'L': self._point_left,  'R': self._point_right,  # point left / right
            '1': self._nod_yes,     '2': self._shake_no,     # yes / no
            '3': self._cheer,                                 # victory sweep
        }

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
        Run the preprogrammed motion mapped to a confirmed character label.

        Edit self._motions (in __init__) and the motion methods below to change
        the repertoire, or replace this with MoveIt / real-robot API calls.

        Args:
            label: The confirmed character, e.g. 'A', '3', 'X'
        """
        motion = self._motions.get(label)
        if motion is None:
            rospy.logwarn('No motion mapped for "%s" — small nod instead.', label)
            self._nod_yes()
            return
        rospy.loginfo('Action "%s" → %s', label,
                      motion.__name__.lstrip('_').replace('_', ' '))
        motion()

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

        # Fire-and-forget: the controller plays the whole trajectory in Gazebo on
        # its own. We don't block on the result because franka_gazebo's effort
        # controller tends to settle just outside the tight goal tolerance, so
        # waiting would stall (and log spurious "did not finish" warnings). Not
        # waiting also lets the next confirmed gesture preempt the current motion.
        self.traj_client.send_goal(goal)
        return True

    # ── Preprogrammed motions ───────────────────────────────────────────────────
    # Each method builds a list of (joint_positions, time_from_start_seconds)
    # waypoints and sends them as one trajectory. Larger time gaps = calmer
    # motion. All joint values stay inside both Panda and FR3 limits.

    @staticmethod
    def _pose(**joints):
        """HOME pose with joint overrides, e.g. _pose(j1=0.8, j6=2.0).

        Keys are j1..j7 (1-indexed joints).
        """
        pose = list(HOME_POSE)
        for key, value in joints.items():
            pose[int(key[1:]) - 1] = value
        return pose

    def _home(self):
        """Return to the neutral elbow-bend pose."""
        self._send_trajectory([(HOME_POSE, 2.5)])

    def _wave(self):
        """Raise the arm and wave the hand (wrist roll) side to side. 👋"""
        ready = self._pose(j2=-0.7, j4=-1.3, j6=1.2)
        a = list(ready); a[6] = 1.6
        b = list(ready); b[6] = -0.1
        self._send_trajectory([
            (ready, 1.6), (a, 2.3), (b, 3.0), (a, 3.7), (b, 4.4), (HOME_POSE, 6.0),
        ])

    def _bow(self):
        """A polite bow forward, hold, then straighten up. 🙇"""
        bow = self._pose(j2=0.5, j4=-2.0, j6=2.4)
        self._send_trajectory([(bow, 2.2), (bow, 3.1), (HOME_POSE, 5.1)])

    def _nod_yes(self):
        """Nod the wrist up and down: yes. ✅"""
        down = self._pose(j6=1.95)
        up   = self._pose(j6=1.20)
        self._send_trajectory([
            (down, 0.9), (up, 1.7), (down, 2.5), (up, 3.3), (HOME_POSE, 4.3),
        ])

    def _shake_no(self):
        """Swing the base left/right: no. ❌"""
        left  = self._pose(j1=0.45)
        right = self._pose(j1=-0.45)
        self._send_trajectory([
            (left, 0.9), (right, 1.7), (left, 2.5), (right, 3.3), (HOME_POSE, 4.3),
        ])

    def _celebrate(self):
        """Arm up, a couple of quick fist pumps. 💪"""
        up   = self._pose(j2=-1.00, j4=-1.1, j6=1.0)
        pump = self._pose(j2=-1.35, j4=-1.1, j6=1.0)
        self._send_trajectory([
            (up, 1.4), (pump, 1.9), (up, 2.4), (pump, 2.9), (up, 3.4), (HOME_POSE, 5.0),
        ])

    def _cheer(self):
        """Arm raised high, sweeping side to side like a victory wave. 🙌"""
        up    = self._pose(j2=-1.2, j4=-0.9, j6=0.9)
        left  = list(up); left[0]  = 0.5
        right = list(up); right[0] = -0.5
        self._send_trajectory([
            (up, 1.6), (left, 2.4), (right, 3.2), (left, 4.0), (HOME_POSE, 5.6),
        ])

    def _dab(self):
        """Strike a dab — arm up and across — hold, then recover. 😎"""
        dab = [-0.7, -0.9, 0.5, -0.7, 0.6, 1.0, 0.4]
        self._send_trajectory([(dab, 1.8), (dab, 2.7), (HOME_POSE, 4.7)])

    def _stretch_up(self):
        """Reach straight up tall, hold, return. 🙆"""
        tall = self._pose(j2=-1.45, j4=-0.45, j6=1.0)
        self._send_trajectory([(tall, 2.3), (tall, 3.2), (HOME_POSE, 5.3)])

    def _point_left(self):
        """Rotate the base to point left. 👈"""
        self._send_trajectory([(self._pose(j1=0.8), 2.5)])

    def _point_right(self):
        """Rotate the base to point right. 👉"""
        self._send_trajectory([(self._pose(j1=-0.8), 2.5)])

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
