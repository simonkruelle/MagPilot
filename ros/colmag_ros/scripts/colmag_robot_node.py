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
  ~time_scale      (float, default 2.0)                           — multiplies all waypoint times. The motion timings
                                                                    below were tuned in Gazebo; at 1.0 'wave' and
                                                                    'celebrate' exceed FR3 joint velocity/acceleration
                                                                    limits and trigger reflex stops on the real arm.
                                                                    2.0 keeps every motion under ~70% of the limits.

Usage:
  rosrun colmag_ros colmag_robot_node.py
  # Drive the simulated FR3:
  rosrun colmag_ros colmag_robot_node.py _dry_run:=false _arm_id:=fr3
  # Drive the simulated Panda:
  rosrun colmag_ros colmag_robot_node.py _dry_run:=false _arm_id:=panda
"""

import time

import numpy as np
import rospy
from std_msgs.msg import Bool, String, Float64

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


# ── Minimal FR3/Panda kinematics for Cartesian digit targets ────────────────
# Mirrored from colmag_draw_node.py (keep in sync). Franka modified-DH (Craig);
# flange row last. Used only to place digits 1..9 on a constant-X line via IK.
_DH = [
    (0.0,     0.333, 0.0),
    (0.0,     0.0,  -np.pi / 2),
    (0.0,     0.316, np.pi / 2),
    (0.0825,  0.0,   np.pi / 2),
    (-0.0825, 0.384, -np.pi / 2),
    (0.0,     0.0,   np.pi / 2),
    (0.088,   0.0,   np.pi / 2),
    (0.0,     0.107, 0.0),
]
# Intersection of Panda and FR3 joint limits — safe on either arm.
_Q_MIN = np.array([-2.7437, -1.7628, -2.8973, -3.0421, -2.8065, 0.5445, -2.8973])
_Q_MAX = np.array([2.7437, 1.7628, 2.8973, -0.1518, 2.8065, 3.7525, 2.8973])


def _dh_T(a, d, alpha, theta):
    ct, st, ca, sa = np.cos(theta), np.sin(theta), np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st, 0, a],
        [st * ca, ct * ca, -sa, -d * sa],
        [st * sa, ct * sa, ca, d * ca],
        [0, 0, 0, 1.0],
    ])


def _fk(q):
    T = np.eye(4)
    for i, (a, d, alpha) in enumerate(_DH):
        T = T @ _dh_T(a, d, alpha, q[i] if i < 7 else 0.0)
    return T


def _jacobian(q, eps=1e-6):
    T0 = _fk(q)
    p0, R0 = T0[:3, 3], T0[:3, :3]
    J = np.zeros((6, 7))
    for i in range(7):
        dq = q.copy(); dq[i] += eps
        Ti = _fk(dq)
        J[:3, i] = (Ti[:3, 3] - p0) / eps
        dR = (Ti[:3, :3] - R0) @ R0.T
        J[3:, i] = np.array([dR[2, 1], dR[0, 2], dR[1, 0]]) / eps
    return J


def _ori_error(R_cur, R_des):
    return 0.5 * (np.cross(R_cur[:, 0], R_des[:, 0])
                  + np.cross(R_cur[:, 1], R_des[:, 1])
                  + np.cross(R_cur[:, 2], R_des[:, 2]))


def _solve_ik(target_p, R_des, q_seed, iters=80, lam=0.03, k_null=0.05):
    """Damped least-squares IK for position + fixed orientation.

    Returns (q, position_error_m).
    """
    q = q_seed.copy()
    home = q_seed.copy()
    for _ in range(iters):
        T = _fk(q)
        ep = target_p - T[:3, 3]
        eo = _ori_error(T[:3, :3], R_des)
        if np.linalg.norm(ep) < 1e-4 and np.linalg.norm(eo) < 1e-3:
            break
        ep = ep * min(1.0, 0.02 / (np.linalg.norm(ep) + 1e-9))
        eo = eo * min(1.0, 0.05 / (np.linalg.norm(eo) + 1e-9))
        e = np.concatenate([ep, eo])
        J = _jacobian(q)
        JJt = J @ J.T + (lam ** 2) * np.eye(6)
        Jpinv = J.T @ np.linalg.inv(JJt)
        null = (np.eye(7) - Jpinv @ J) @ (k_null * (home - q))
        q = np.clip(q + J.T @ np.linalg.solve(JJt, e) + null, _Q_MIN, _Q_MAX)
    T = _fk(q)
    return q, float(np.linalg.norm(target_p - T[:3, 3]))


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
        self.time_scale    = float(rospy.get_param('~time_scale', 2.0))
        if self.time_scale <= 0.0:
            raise ValueError('~time_scale must be > 0')
        if self.time_scale < 1.0:
            rospy.logwarn('~time_scale=%.2f is faster than the Gazebo-tuned timings; '
                          'expect reflex stops on a real arm.', self.time_scale)

        self.joint_names = ['{}_joint{}'.format(self.arm_id, i) for i in range(1, 8)]

        # ── Digit test line (constant-X, IK-placed) ───────────────────────────
        # Digits 1..9 park the arm at nine evenly spaced Y slots at the home X/Z
        # and pen orientation, so each recognised digit lands at a distinct,
        # ordered spot on a straight lateral line — easy to tell apart when
        # checking the digit classifier.
        self.digit_line_span = float(rospy.get_param('~digit_line_span', 0.22))
        self.digit_reach_tol = float(rospy.get_param('~digit_reach_tol', 0.03))
        _T_home = _fk(np.array(HOME_POSE, dtype=float))
        self._pen_R   = _T_home[:3, :3]
        self._home_x  = float(_T_home[0, 3])   # constant-X plane of the line
        self._digit_z = float(_T_home[2, 3])   # line height (home height)

        # ── Gesture → motion map ──────────────────────────────────────────────
        # Edit this (and the motion methods below) to change the robot's tricks.
        self._motions = {
            '0': self._home,        'X': self._home,         # reset / cancel
            'A': self._wave,        'B': self._bow,          # wave, bow
            'C': self._celebrate,   'D': self._dab,          # fist pumps, dab
            'U': self._stretch_up,                            # reach tall
            'L': self._point_left,  'R': self._point_right,  # point left / right
        }
        # Digits 1..9 → nine positions on a constant-X line (classifier test).
        self._motions.update({
            str(n): (lambda n=n: self._go_to_digit(n)) for n in range(1, 10)
        })

        # ── Classifier state ──────────────────────────────────────────────────
        self.last_label      = None
        self.last_confidence = 0.0
        # While the teleop/draw mode is active (colmag_draw_node follows the
        # cursor), gesture motions are suspended so the two never fight.
        self.teleop_active   = False

        # ── Trajectory action client (only when actually moving a robot) ──────
        self.traj_client = None
        if not self.dry_run:
            self._setup_trajectory_client()

        # Trajectories are fire-and-forget (see _send_trajectory), so without
        # this hook the controller keeps playing the rest of a gesture after
        # Ctrl-C. Cancelling makes the arm stop and hold where it is.
        rospy.on_shutdown(self._cancel_on_shutdown)

        # ── Subscribers ───────────────────────────────────────────────────────
        # /colmag/draw_active is a LATCHED Bool from colmag_draw_node telling
        # whether teleop currently owns the arm — reliable even if this node
        # starts later and missed the 'robot:teleop' command.
        self._draw_active = None   # None = unknown (no draw node heard from)
        rospy.Subscriber('/colmag/command',     String,  self.on_command)
        rospy.Subscriber('/colmag/classifier',  String,  self.on_classifier)
        rospy.Subscriber('/colmag/confidence',  Float64, self.on_confidence)
        rospy.Subscriber('/colmag/draw_active', Bool,    self.on_draw_active)

        mode = ('DRY RUN (no robot actions)' if self.dry_run
                else 'LIVE (arm_id={}, controller={}, time_scale={:.1f})'.format(
                    self.arm_id, self.controller, self.time_scale))
        rospy.loginfo('colmag_robot_node ready | mode={}'.format(mode))

        # Home the arm on startup so every session begins from a known pose —
        # unless teleop is ACTIVE (draw node streaming): both nodes drive the
        # same trajectory controller and would preempt each other. An idle draw
        # node is fine (it only streams once Teleop is enabled), so starting
        # both nodes together still homes. ~home_on_start:=false disables.
        self.home_on_start = bool(rospy.get_param('~home_on_start', True))
        if not self.dry_run and self.home_on_start:
            # Give a (possibly simultaneously launched) draw node a moment to
            # latch its state; wall-clock sleep so sim-time can't stall this.
            deadline = time.monotonic() + 2.0
            while self._draw_active is None and time.monotonic() < deadline:
                time.sleep(0.1)
            if self._draw_active or (self._draw_active is None
                                     and self._draw_node_running()):
                rospy.logwarn('Skipping startup homing: teleop draw node %s. '
                              'Confirm a "0" gesture to home instead.',
                              'is ACTIVE' if self._draw_active
                              else 'state unknown (old version?)')
            else:
                rospy.loginfo('Homing arm on startup...')
                self._home()

    def on_draw_active(self, msg):
        self._draw_active = bool(msg.data)
        self.teleop_active = self._draw_active

    @staticmethod
    def _draw_node_running():
        """True if the colmag draw/teleop node is registered with the master."""
        try:
            import rosgraph
            rosgraph.Master('/colmag_robot_node').lookupNode('/colmag_draw_node')
            return True
        except Exception:
            return False

    def _cancel_on_shutdown(self):
        if self.traj_client is None:
            return
        try:
            self.traj_client.cancel_all_goals()
            rospy.logwarn('Shutdown: cancelled active arm trajectory — the arm '
                          'stops and holds its current position.')
        except Exception as exc:
            rospy.logwarn('Shutdown: could not cancel trajectory: %s', exc)

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
            if self.teleop_active:
                rospy.loginfo('Teleop active — ignoring confirmed gesture "%s".',
                              self.last_label)
                return
            label = self.last_label
            conf  = self.last_confidence * 100
            rospy.loginfo(
                'CONFIRMED gesture: "%s" (%.1f%%) — %s',
                label, conf, '(dry run)' if self.dry_run else 'executing'
            )
            if not self.dry_run:
                self.execute_command(label)
        elif cmd == 'robot:teleop':
            self.teleop_active = True
            rospy.loginfo('Teleop mode active — gesture motions suspended '
                          '(press Letters/Digits/Signs to resume).')
        elif cmd == 'letter_detection':
            self.teleop_active = False
            rospy.loginfo('Mode → letters OCR')
        elif cmd == 'number_detection':
            self.teleop_active = False
            rospy.loginfo('Mode → digits OCR')
        elif cmd == 'symbol_detection':
            self.teleop_active = False
            rospy.loginfo('Mode → signs / shape-recognition placeholder')
        elif cmd == 'canvas:reset':
            rospy.loginfo('Canvas reset')
        elif cmd.startswith(('gripper:', 'plane:', 'rotate:')):
            pass  # teleop-mode commands, handled by colmag_draw_node
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
                    Times are multiplied by ~time_scale before sending, so the
                    motion methods below stay written in their original
                    Gazebo-tuned timings.
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
            pt.time_from_start = rospy.Duration(t * self.time_scale)
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

    def _go_to_digit(self, n):
        """Move the end-effector to the n-th slot on a constant-X line (n=1..9).

        Nine evenly spaced Y positions at the home X and Z with the home pen
        orientation, placed by IK — so each recognised digit parks the arm at a
        distinct, ordered spot on a straight lateral line, easy to differentiate.
        """
        n = max(1, min(9, int(round(n))))
        span = self.digit_line_span
        # digit 1 → +span (robot's left), digit 9 → -span (right); linear.
        y = span - 2.0 * span * (n - 1) / 8.0
        target = np.array([self._home_x, y, self._digit_z])
        q, err = _solve_ik(target, self._pen_R, np.array(HOME_POSE, dtype=float))
        if err > self.digit_reach_tol:
            rospy.logwarn('Digit %d target y=%+.3f unreachable (IK err %.0f mm) — skipping.',
                          n, y, err * 1000.0)
            return
        rospy.loginfo('Digit %d → line slot y=%+.3f m (x=%.2f, z=%.2f), IK err %.1f mm',
                      n, y, self._home_x, self._digit_z, err * 1000.0)
        self._send_trajectory([(list(q), self.move_duration)])

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
        """Arm up, a couple of quick elbow pumps. 💪

        Pumps with the elbow (j4), not the shoulder: the old j2 pump swung the
        wrist ~0.47 m behind the base (FK-checked). Keeping j2 fixed keeps the
        whole motion inside the same envelope as _wave.
        """
        up   = self._pose(j2=-0.7, j4=-1.70, j6=1.0)
        pump = self._pose(j2=-0.7, j4=-1.25, j6=1.0)
        self._send_trajectory([
            (up, 1.4), (pump, 1.9), (up, 2.4), (pump, 2.9), (up, 3.4), (HOME_POSE, 5.0),
        ])

    def _cheer(self):
        """Arm raised high, sweeping side to side like a victory wave. 🙌"""
        # j2=-1.2/j4=-0.9 put the wrist ~0.46 m behind the base; this raise
        # stays in front (same geometry as _wave's ready pose).
        up    = self._pose(j2=-0.7, j4=-1.3, j6=0.9)
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
        """Reach straight up tall, hold, return. 🙆

        Upper arm near vertical with the elbow bent forward: reaches higher
        (wrist z ≈ 1.0 m vs 0.59 m before) and stays entirely in front of the
        base — the old j2=-1.45/j4=-0.45 pose reached ~0.67 m behind it.
        """
        tall = self._pose(j2=-0.2, j4=-0.9, j6=1.0)
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
