#!/usr/bin/env python3
"""
colmag_draw_node.py — Cartesian "draw in the air" teleoperation mode.

Instead of classifying a gesture, this node makes the end-effector trace the
cursor/magnet: the 2D position on /colmag/pose is mapped onto a flat plane in
front of the robot, and the arm follows it in real time (like drawing on an
invisible whiteboard). It is an alternative to the classify-and-gesture pipeline
in colmag_robot_node.py.

How it works
  - /colmag/pose (PoseStamped, colmag_base frame) gives magnet/cursor x,y in
    metres, clipped by the reader/sensor to +-input_extent (default 0.05 m).
  - (x, y) are scaled onto the active control layer in a bounded workspace box:
    horizontal = move in x/y, vertical = move in y/z, rotate = hold position and
    rotate the gripper/wrist from cursor x.
  - A damped-least-squares IK solves for joint angles that put the flange at that
    point while holding a fixed "pen" orientation (the natural orientation at the
    plane centre), seeded from the last commanded pose for continuity.
  - The solution is streamed to the same JointTrajectoryController used
    elsewhere (default position_joint_trajectory_controller), so no new
    controller or MoveIt is required. Segments are stamped on a uniform time
    grid (send-time jitter must not modulate segment durations), carry the
    exact slope of the commanded sequence as velocity, and include a second
    look-ahead point so a late tick continues the motion instead of braking.

Mode switching (via /colmag/command, i.e. the touchpad UI buttons)
  - 'robot:teleop'  (Teleop button, top of the left pad)  -> following ENABLED
  - 'letter_detection' / 'number_detection' / 'symbol_detection'
    (Letters/Digits/Signs buttons)                        -> following DISABLED
  - 'gripper:toggle' / 'gripper:open' / 'gripper:close'
    (Shift+G in the touchpad UI)                          -> franka_gripper action
  - 'rotate:reference' / 'rotate:set:<degrees>'           -> absolute magnet
    twist target relative to the captured wrist angle
  - 'plane:toggle' / 'layer:next' (Shift+V / Layer button) -> cycle through
    horizontal (x/y), vertical (y/z), and rotate-wrist. Coordinates not owned by
    the active layer keep their last value, so the layers together position and
    orient the flange inside the workspace box.
  /colmag/draw_enable (std_msgs/Bool) remains as a manual override.

The plane is horizontal by default (like a tabletop floating in front of the
robot), oriented for someone standing IN FRONT of the robot: cursor up moves to
the plane's far edge (toward the robot), cursor right to the viewer's right, at
fixed height plane_center_z. Set plane_orientation:=vertical for an upright
canvas instead. NOTE: the IK targets the FLANGE; with the hand mounted the
fingertips sit ~0.10-0.16 m below it.

SAFETY
  - Defaults to dry_run: logs targets and IK residuals, sends nothing. Validate
    the mapping in Gazebo (Stage 3 style) before ever setting dry_run:=false.
  - Until enabled (Teleop button or /colmag/draw_enable) the arm holds position.
    On disable or shutdown, active goals are cancelled.
  - A minimum-jerk quintic S-curve bounds Cartesian speed, acceleration and
    jerk; enabling teleop far from the cursor produces a smooth approach.
  - Each update additionally clamps per-joint change to max_joint_step.
  - IK solutions with residual > max_reach_error are rejected; the target holds
    at the last reachable point instead of drifting through dead zones.
  - If pose.z is farther than lift_gate_z (default 0.15 m), following pauses and
    the controller holds. Lowering the magnet below the hysteresis band resumes.

Parameters (see below for defaults). Usage:
  # Gazebo rehearsal, touchpad input, no real motion until Teleop is pressed:
  rosrun colmag_ros colmag_draw_node.py _dry_run:=false _arm_id:=fr3
"""

import sys
import time

import numpy as np
import rospy
from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

sys.path.insert(0, '/colmag')
from colmag.control_mapping import (  # noqa: E402
    EE_HEIGHT_MAX_M,
    EE_HEIGHT_MIN_M,
    MAGNET_HEIGHT_GAIN,
    MAGNET_HEIGHT_MAX_M,
    MAGNET_HEIGHT_MIN_M,
    map_magnet_height,
)

try:
    import actionlib
    from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    _TRAJ_AVAILABLE = True
except ImportError:
    _TRAJ_AVAILABLE = False

try:
    from franka_gripper.msg import (GraspAction, GraspGoal,
                                    MoveAction, MoveGoal,
                                    StopAction, StopGoal)
    _GRIPPER_AVAILABLE = True
except ImportError:
    _GRIPPER_AVAILABLE = False

# Franka modified-DH (Craig convention); flange row last (theta = 0).
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
# INTERSECTION of Panda and FR3 joint limits — safe on either arm. (They are
# not supersets of each other: FR3 is narrower on j1/j4max and its j6 minimum
# is 0.5445 vs Panda's -0.0175.)
_Q_MIN = np.array([-2.7437, -1.7628, -2.8973, -3.0421, -2.8065, 0.5445, -2.8973])
_Q_MAX = np.array([2.7437, 1.7628, 2.8973, -0.1518, 2.8065, 3.7525, 2.8973])
_HOME = np.array([0.0, -0.785398163, 0.0, -2.35619449, 0.0, 1.57079632679, 0.785398163397])

# Responsive but still conservative MagPilot defaults. The target and
# height filters remove sensor chatter; the quintic planner remains the
# authoritative speed, acceleration and jerk limiter.
DEFAULT_TARGET_SMOOTHING_TAU = 0.04
DEFAULT_HEIGHT_SMOOTHING_TAU = 0.25
DEFAULT_HEIGHT_SMOOTHING_TAU_FAR = 0.65
DEFAULT_MAX_LINEAR_SPEED = 0.12
DEFAULT_MAX_LINEAR_ACCELERATION = 0.30
DEFAULT_MAX_LINEAR_JERK = 1.50
DEFAULT_S_CURVE_MIN_DURATION = 0.50


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


def _jacobian(q):
    """Geometric Jacobian (world frame) from one pass over the DH chain.

    Joint i rotates about the z-axis of its own frame and the frame origin
    lies on that axis, so J_lin[:, i] = z_i x (p_e - p_i), J_ang[:, i] = z_i.
    IK runs twice inside the 33 ms streaming tick, so the Jacobian cannot
    afford the seven extra FK evaluations of a finite-difference version.
    """
    T = np.eye(4)
    axes = np.zeros((7, 3))
    origins = np.zeros((7, 3))
    for i, (a, d, alpha) in enumerate(_DH):
        T = T @ _dh_T(a, d, alpha, q[i] if i < 7 else 0.0)
        if i < 7:
            axes[i] = T[:3, 2]
            origins[i] = T[:3, 3]
    p_e = T[:3, 3]
    J = np.zeros((6, 7))
    J[:3] = np.cross(axes, p_e - origins).T
    J[3:] = axes.T
    return J


def _limit_norm(vector, limit):
    """Return vector with its Euclidean norm limited to a positive bound."""
    vector = np.asarray(vector, dtype=float)
    norm = float(np.linalg.norm(vector))
    if norm <= float(limit) or norm <= 1e-12:
        return vector.copy()
    return vector * (float(limit) / norm)


def _s_curve_step(position, velocity, acceleration, desired, dt,
                  min_duration, max_speed, max_acceleration, max_jerk):
    """Advance one receding-horizon minimum-jerk quintic S-curve step."""
    position = np.asarray(position, dtype=float)
    velocity = np.asarray(velocity, dtype=float)
    acceleration = np.asarray(acceleration, dtype=float)
    desired = np.asarray(desired, dtype=float)
    dt = float(dt)
    distance = float(np.linalg.norm(desired - position))
    if (distance <= 0.00025
            and np.linalg.norm(velocity) <= max_acceleration * dt
            and np.linalg.norm(acceleration) <= max_jerk * dt):
        return desired.copy(), np.zeros_like(velocity), np.zeros_like(acceleration)
    duration = max(
        float(min_duration),
        1.875 * distance / float(max_speed),
        np.sqrt(5.8 * distance / float(max_acceleration)),
        np.cbrt(60.0 * distance / float(max_jerk)))

    def sample(horizon):
        c0 = position
        c1 = velocity
        c2 = 0.5 * acceleration
        rhs = np.vstack([
            desired - (c0 + c1 * horizon + c2 * horizon ** 2),
            -c1 - 2.0 * c2 * horizon,
            -2.0 * c2,
        ])
        matrix = np.array([
            [horizon ** 3, horizon ** 4, horizon ** 5],
            [3.0 * horizon ** 2, 4.0 * horizon ** 3,
             5.0 * horizon ** 4],
            [6.0 * horizon, 12.0 * horizon ** 2,
             20.0 * horizon ** 3],
        ])
        c3, c4, c5 = np.linalg.solve(matrix, rhs)
        t = min(dt, horizon)
        next_position = (c0 + c1 * t + c2 * t ** 2 + c3 * t ** 3
                         + c4 * t ** 4 + c5 * t ** 5)
        next_velocity = (c1 + 2.0 * c2 * t + 3.0 * c3 * t ** 2
                         + 4.0 * c4 * t ** 3 + 5.0 * c5 * t ** 4)
        next_acceleration = (2.0 * c2 + 6.0 * c3 * t
                             + 12.0 * c4 * t ** 2
                             + 20.0 * c5 * t ** 3)
        start_jerk = 6.0 * c3
        end_jerk = (6.0 * c3 + 24.0 * c4 * t
                    + 60.0 * c5 * t ** 2)
        return (next_position, next_velocity, next_acceleration,
                max(np.linalg.norm(start_jerk), np.linalg.norm(end_jerk)))

    # Non-zero incoming velocity/acceleration can require a longer horizon than
    # the rest-to-rest estimates above. Expand it until this first control step
    # satisfies every derivative bound.
    for _ in range(12):
        next_position, next_velocity, next_acceleration, jerk_norm = sample(
            duration)
        if (np.linalg.norm(next_velocity) <= max_speed + 1e-12
                and np.linalg.norm(next_acceleration)
                <= max_acceleration + 1e-12
                and jerk_norm <= max_jerk + 1e-12):
            return next_position, next_velocity, next_acceleration
        duration *= 1.35

    return (next_position,
            _limit_norm(next_velocity, max_speed),
            _limit_norm(next_acceleration, max_acceleration))


def _ori_error(R_cur, R_des):
    return 0.5 * (np.cross(R_cur[:, 0], R_des[:, 0])
                  + np.cross(R_cur[:, 1], R_des[:, 1])
                  + np.cross(R_cur[:, 2], R_des[:, 2]))


def _solve_ik(target_p, R_des, q_seed, iters=60, lam=0.03, k_null=0.05):
    """Damped least squares IK for position + fixed orientation.

    Returns (q, position_error_m). Small, error-clamped steps keep it stable;
    a gentle nullspace term biases the redundant elbow toward HOME.
    """
    q = q_seed.copy()
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
        null = (np.eye(7) - Jpinv @ J) @ (k_null * (_HOME - q))
        q = np.clip(q + Jpinv @ e + null, _Q_MIN, _Q_MAX)
    T = _fk(q)
    return q, float(np.linalg.norm(target_p - T[:3, 3]))


class ColmagDrawNode:
    def __init__(self):
        rospy.init_node('colmag_draw_node', anonymous=False)

        self.arm_id      = rospy.get_param('~arm_id', 'fr3')
        # Default matches franka_gazebo's sim_controllers.yaml (like
        # colmag_robot_node); on the real arm pass
        # _arm_controller:=position_joint_trajectory_controller.
        self.controller  = rospy.get_param('~arm_controller', 'effort_joint_trajectory_controller')
        self.dry_run     = bool(rospy.get_param('~dry_run', True))

        # Input range of /colmag/pose x,y (matches the reader/classifier extent).
        self.input_extent = float(rospy.get_param('~input_extent', 0.05))
        # Drawing plane in the robot base frame. 'horizontal' (default): a
        # tabletop-like plane at height plane_center_z, cursor up = further away.
        # 'vertical': an upright canvas facing the robot, cursor up = higher.
        self.plane_orientation = str(rospy.get_param('~plane_orientation', 'horizontal')).lower()
        if self.plane_orientation not in ('horizontal', 'vertical'):
            raise ValueError("~plane_orientation must be 'horizontal' or 'vertical'")
        self.layer_modes = ('horizontal', 'vertical', 'rotate')
        # Workspace box defaults were validated offline against the FR3/Panda
        # joint-limit intersection with the gripper-down orientation: a dense
        # 729-point sweep of x 0.30-0.60, y ±0.30, z 0.12-0.68 has only the four
        # extreme far-top-edge points unreachable (safely skipped by the
        # max_reach_error guard). The bottom layer (z=0.12, fingertips at about
        # base height with the hand mounted) is fully reachable for pick-ups.
        self.center = np.array([
            float(rospy.get_param('~plane_center_x', 0.45)),
            float(rospy.get_param('~plane_center_y', 0.0)),
            float(rospy.get_param('~plane_center_z', 0.40)),
        ])
        self.half_w = 0.5 * float(rospy.get_param('~plane_width', 0.60))
        self.half_d = 0.5 * float(rospy.get_param('~plane_depth', 0.30))
        self.half_h = 0.5 * float(rospy.get_param('~plane_height', 0.56))
        # Axis signs to flip the mapping if it feels mirrored.
        self.sign_w = float(rospy.get_param('~map_x_sign', 1.0))
        self.sign_h = float(rospy.get_param('~map_y_sign', 1.0))

        # Magnet height → flange height (horizontal plane). The 7 mm lower
        # bound accounts for the board between magnet and sensors. A saturating
        # curve maps 7..150 mm to the full vertical workspace while reducing
        # sensitivity where high-distance measurements are least certain.
        self.height_from_magnet = bool(rospy.get_param('~height_from_magnet', True))
        self.height_sensor_min = float(rospy.get_param(
            '~height_sensor_min', MAGNET_HEIGHT_MIN_M))
        self.height_sensor_max = float(rospy.get_param(
            '~height_sensor_max', MAGNET_HEIGHT_MAX_M))
        self.height_gain = float(rospy.get_param(
            '~height_gain', MAGNET_HEIGHT_GAIN))
        # Low-pass time constant on the measured height. The dipole field a
        # magnet emits falls off as ~1/r^3, so a fixed measurement noise turns
        # into a height error that grows steeply with distance: readings near
        # the board are crisp, readings far from it are noisy. The filter is
        # therefore distance-adaptive — it interpolates from the responsive
        # near value at the board to the heavier far value at the top of the
        # range, so lifting the magnet high is smoothed hard (where the sensor
        # is least certain) while fine work near the board stays quick. Set
        # ~height_smoothing_tau_far == ~height_smoothing_tau for the old fixed
        # behaviour.
        self.height_smoothing_tau = max(0.0, float(rospy.get_param(
            '~height_smoothing_tau', DEFAULT_HEIGHT_SMOOTHING_TAU)))
        self.height_smoothing_tau_far = max(0.0, float(rospy.get_param(
            '~height_smoothing_tau_far', DEFAULT_HEIGHT_SMOOTHING_TAU_FAR)))
        self.height_noise_deadband = abs(float(rospy.get_param(
            '~height_noise_deadband', 0.001)))
        self.height_ee_min = float(rospy.get_param(
            '~height_ee_min', EE_HEIGHT_MIN_M))
        self.height_ee_max = float(rospy.get_param(
            '~height_ee_max', EE_HEIGHT_MAX_M))
        if self.height_sensor_max <= self.height_sensor_min:
            raise ValueError('~height_sensor_max must exceed ~height_sensor_min')
        if self.height_ee_max <= self.height_ee_min:
            raise ValueError('~height_ee_max must exceed ~height_ee_min')
        if self.height_gain < 0.0:
            raise ValueError('~height_gain must be non-negative')

        self.update_rate    = float(rospy.get_param('~update_rate', 30.0))
        self.command_time   = float(rospy.get_param('~command_time', 0.10))
        # Low-pass on the cursor target (first-order time constant, seconds).
        # Smooths pointer jitter and direction flips before they reach IK; the
        # velocity continuity below then carries the smoothness into the spline.
        self.target_tau     = float(rospy.get_param(
            '~target_smoothing_tau', DEFAULT_TARGET_SMOOTHING_TAU))
        self.max_joint_step = float(rospy.get_param('~max_joint_step', 0.05))
        self.max_reach_error = float(rospy.get_param('~max_reach_error', 0.005))
        self.start_enabled  = bool(rospy.get_param('~start_enabled', False))
        # Receding-horizon minimum-jerk spline limits. Together these produce
        # an S-shaped speed profile without changing the position controller.
        self.max_lin_speed  = float(rospy.get_param(
            '~max_linear_speed', DEFAULT_MAX_LINEAR_SPEED))
        self.max_lin_acceleration = float(rospy.get_param(
            '~max_linear_acceleration', DEFAULT_MAX_LINEAR_ACCELERATION))
        self.max_lin_jerk = float(rospy.get_param(
            '~max_linear_jerk', DEFAULT_MAX_LINEAR_JERK))
        self.s_curve_min_duration = float(rospy.get_param(
            '~s_curve_min_duration', DEFAULT_S_CURVE_MIN_DURATION))
        if (self.max_lin_speed <= 0.0
                or self.max_lin_acceleration <= 0.0
                or self.max_lin_jerk <= 0.0
                or self.s_curve_min_duration <= 0.0):
            raise ValueError('S-curve speed, acceleration, jerk and duration '
                             'parameters must be positive')
        # Real magnet input: lifting the magnet away from the board should pause
        # following so the user can travel to taskbar buttons without dragging
        # the robot. Set lift_gate_z<=0 to disable.
        self.lift_gate_z = float(rospy.get_param(
            '~lift_gate_z', self.height_sensor_max))
        self.lift_gate_hysteresis = abs(float(rospy.get_param(
            '~lift_gate_hysteresis', 0.010)))
        self.lift_reengage_z = max(0.0, self.lift_gate_z - self.lift_gate_hysteresis)
        self._lift_paused = False
        # Grasp force in N (Franka hand: up to 70 N continuous). 20 N holds
        # light objects; raise it for heavier or slippery ones.
        self.grasp_force    = float(rospy.get_param('~grasp_force', 20.0))

        if self.command_time <= 1.0 / self.update_rate:
            rospy.logwarn('command_time (%.3f) <= update period (%.3f); raising it '
                          'to avoid starving the controller.',
                          self.command_time, 1.0 / self.update_rate)
            self.command_time = 2.0 / self.update_rate

        self.joint_names = ['{}_joint{}'.format(self.arm_id, i) for i in range(1, 8)]

        # Freeze the pen orientation as the natural orientation with the flange at
        # the plane centre (validated to be reachable and IK-stable there).
        # Runtime layer mode (Shift+V / Layer toggles it). Horizontal drives
        # (x, y), vertical drives (y, z), rotate turns the wrist at fixed
        # position — together they reach and orient any point in the box.
        self.plane_mode = self.plane_orientation
        self.box_lo = self.center - np.array([self.half_d, self.half_w, self.half_h])
        self.box_hi = self.center + np.array([self.half_d, self.half_w, self.half_h])

        q_ready, err0 = _solve_ik(self.center, _fk(_HOME)[:3, :3], _HOME.copy(),
                                  iters=500, k_null=0.15)
        self.R_pen = _fk(q_ready)[:3, :3]
        # Gripper rotation about the tool axis (Shift+Left/Right in the UI).
        # ±92 deg covers every grasp orientation (two-finger gripper repeats
        # every 180 deg) while keeping joint 7 well inside its limits.
        self.R_pen0 = self.R_pen.copy()
        self.pen_yaw = 0.0
        self._magnet_twist_base_yaw = self.pen_yaw
        self.rotate_step = np.radians(float(rospy.get_param('~rotate_step_deg', 5.0)))
        self.max_pen_yaw = np.radians(float(rospy.get_param('~max_rotate_deg', 92.0)))
        # Smooth hold-to-rotate: while 'rotate:hold:*' keep-alives are fresh,
        # yaw is integrated at this constant rate inside the tick loop.
        self.rotate_speed = np.radians(float(rospy.get_param('~rotate_speed_deg', 35.0)))
        self._rot_dir = 0          # -1 ccw / 0 off / +1 cw
        self._rot_deadline = 0.0   # watchdog: stops if keep-alives cease
        self._rotate_layer_ref_u = None
        self._rotate_layer_ref_yaw = self.pen_yaw
        self.q_ready = q_ready
        self.q_cmd = q_ready.copy()   # last commanded joints (IK warm seed)
        # Persistent 3D target; the cursor only overwrites the coordinates the
        # active layer controls, so the others are held across layer switches.
        self.p_target = _fk(self.q_cmd)[:3, 3].copy()
        rospy.loginfo('Draw plane centre reachable within %.2f mm; pen orientation frozen.',
                      err0 * 1000)

        self.enabled = self.start_enabled
        self.latest_pose = None
        self.latest_joints = None
        self.traj_client = None
        self.traj_pub = None
        self._q_sent = None
        # Velocity continuity for streamed segments: each trajectory point
        # carries the exact slope of the commanded joint sequence instead of
        # 0, so the controller splines THROUGH the waypoints rather than
        # braking to a stop at each one — that stop-start cycle is what made
        # streaming sound and feel rougher than the long gesture trajectories.
        self._desired_filt = None
        self._height_filtered_m = None
        self._cart_velocity = np.zeros(3)
        self._cart_acceleration = np.zeros(3)
        self._qd_cmd = np.zeros(7)
        # Uniform time base for streamed segments (see _next_stream_stamp).
        self._stream_stamp = None
        # While a take-over approach trajectory plays (see
        # _takeover_approach), streaming ticks stay suspended until this time.
        self._stream_hold_until = 0.0
        self._grip_clients = None
        self._grip_closed = False
        # Return the arm to the neutral home pose whenever teleop is exited
        # (Draw button / Shift+E / letter mode). ~home_on_exit:=false to hold.
        self.home_on_exit = bool(rospy.get_param('~home_on_exit', True))
        if not self.dry_run:
            self._setup_trajectory_client()

        self.joint_state_topic = rospy.get_param(
            '~joint_state_topic', '/franka_state_controller/joint_states')

        # Latched teleop-state broadcast: lets colmag_robot_node (and any other
        # arm user) know whether teleop currently owns the arm — even if that
        # node starts later and missed the 'robot:teleop' command.
        self._active_pub = rospy.Publisher('/colmag/draw_active', Bool,
                                           queue_size=1, latch=True)
        self._publish_active()
        # Authoritative gripper state for the UI (gyro readout + tilt hysteresis).
        self._grip_pub = rospy.Publisher('/colmag/gripper_closed', Bool,
                                         queue_size=1, latch=True)
        self._grip_pub.publish(Bool(data=self._grip_closed))

        rospy.on_shutdown(self._cancel_on_shutdown)
        rospy.Subscriber('/colmag/pose', PoseStamped, self._on_pose)
        rospy.Subscriber('/colmag/draw_enable', Bool, self._on_enable)
        rospy.Subscriber('/colmag/command', String, self._on_command)
        rospy.Subscriber(self.joint_state_topic, JointState, self._on_joint_state)
        rospy.Timer(rospy.Duration(1.0 / self.update_rate), self._on_tick)

        lift_text = ('off' if self.lift_gate_z <= 0.0
                     else 'pause above %.1f cm, resume below %.1f cm'
                     % (100.0 * self.lift_gate_z, 100.0 * self.lift_reengage_z))
        rospy.loginfo('colmag_draw_node ready | mode=%s | layer=%s | workspace %.2fx%.2fx%.2f m @ %s | '
                      'S-curve v/a/j=%.2f/%.2f/%.2f | lift_gate=%s | enabled=%s',
                      'DRY RUN' if self.dry_run else 'LIVE (%s)' % self.controller,
                      self.plane_mode, 2 * self.half_d, 2 * self.half_w,
                      2 * self.half_h, np.round(self.center, 2),
                      self.max_lin_speed, self.max_lin_acceleration,
                      self.max_lin_jerk,
                      lift_text, self.enabled)
        if not self.dry_run and not self.enabled:
            rospy.loginfo('Press the Teleop button in the touchpad UI (or publish '
                          'true to /colmag/draw_enable) to start following.')

    def _setup_trajectory_client(self):
        if not _TRAJ_AVAILABLE:
            rospy.logwarn('control_msgs/actionlib not importable — cannot drive the arm. '
                          'Build the image with INSTALL_GAZEBO=1.')
            return
        ns = '/{}/follow_joint_trajectory'.format(self.controller)
        rospy.loginfo('Connecting to %s ...', ns)
        # The action client is used ONLY to cancel goals (ours at shutdown,
        # gesture goals at teleop take-over). Streaming teleop points through
        # the action interface at 15 Hz floods actionlib's goal bookkeeping
        # ("transition callback on a goal handle that we're not tracking"), so
        # points are streamed via the controller's command TOPIC instead, which
        # joint_trajectory_controller provides exactly for this.
        client = actionlib.SimpleActionClient(ns, FollowJointTrajectoryAction)
        if client.wait_for_server(rospy.Duration(10.0)):
            self.traj_client = client
            self.traj_pub = rospy.Publisher('/{}/command'.format(self.controller),
                                            JointTrajectory, queue_size=1)
            rospy.loginfo('Trajectory controller connected (streaming via %s/command).',
                          self.controller)
        else:
            rospy.logwarn('Trajectory server %s not available. Is the arm launched '
                          'with controller:=%s ?', ns, self.controller)

    def _next_stream_stamp(self, now):
        """Stamp streamed segments on a uniform grid, not with send time.

        The waypoint positions advance by exactly one spline step per tick,
        so their deadlines must advance by exactly one period too. Stamping
        with wall-clock send times lets Python timer/IK jitter (several ms
        per tick) modulate the segment durations, and the controller executes
        that as velocity ripple at the streaming rate — felt as vibration
        during motion even though the position sequence itself is smooth.

        Re-anchoring is forward-only: when a tick was skipped (hold,
        unreachable target, idle) the wall clock runs more than a period past
        the grid, so the grid jumps forward to it. A small *lead* (the grid
        ahead of the wall clock, from an early-firing timer) is kept as-is —
        snapping the stamp backward there would place a segment before its
        predecessor and make the controller drop or re-splice it, the very
        jolt this grid removes. rospy.Timer targets absolute tick times, so
        the lead stays bounded within one period instead of accumulating.
        """
        period = 1.0 / self.update_rate
        if self._stream_stamp is None or (now - self._stream_stamp).to_sec() > period:
            self._stream_stamp = now
        stamp = self._stream_stamp
        self._stream_stamp = stamp + rospy.Duration(period)
        return stamp

    def _halt_streaming(self):
        """Stop any in-flight streamed motion: an empty trajectory makes the
        controller drop queued segments and hold the current position."""
        if self.traj_pub is not None:
            stop = JointTrajectory()
            stop.header.stamp = rospy.Time.now()
            self.traj_pub.publish(stop)

    def _cancel_on_shutdown(self):
        try:
            self._halt_streaming()
            if self.traj_client is not None:
                self.traj_client.cancel_all_goals()
                rospy.logwarn('Shutdown: halted arm motion — arm holds position.')
        except Exception as exc:
            rospy.logwarn('Shutdown: could not halt motion: %s', exc)

    def _go_home(self, duration=3.0):
        """Send the arm back to the neutral elbow-bend pose (teleop exit)."""
        if self.dry_run or self.traj_client is None:
            return
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = list(self.joint_names)
        pt = JointTrajectoryPoint()
        pt.positions = list(_HOME)
        pt.velocities = [0.0] * len(self.joint_names)
        pt.time_from_start = rospy.Duration(duration)
        goal.trajectory.points = [pt]
        goal.trajectory.header.stamp = rospy.Time.now()
        self.traj_client.send_goal(goal)
        # Resync command state to home so a later re-enable starts clean.
        self.q_cmd = _HOME.copy()
        self.p_target = _fk(self.q_cmd)[:3, 3].copy()
        self._q_sent = None
        self._desired_filt = None
        self._height_filtered_m = None
        self._cart_velocity = np.zeros(3)
        self._cart_acceleration = np.zeros(3)
        self._qd_cmd = np.zeros(7)
        self.pen_yaw = 0.0
        self.R_pen = self.R_pen0.copy()
        self._magnet_twist_base_yaw = self.pen_yaw
        self._rotate_layer_ref_yaw = 0.0
        rospy.loginfo('Teleop exit: homing the arm (%.1fs).', duration)

    def _needs_takeover_approach(self):
        """True when the arm's current posture cannot be followed directly.

        Taking over mid-gesture (e.g. Teleop pressed during the wave) leaves
        q_cmd at a pose whose wrist orientation is far from the frozen pen
        orientation, often near the workspace edge — from that seed the plane
        IK never converges and the tick would hold forever (observed live:
        'IK residual 5.3 mm — holding' loop after enabling mid-wave)."""
        T = _fk(self.q_cmd)
        ori_gap = float(np.linalg.norm(_ori_error(T[:3, :3], self.R_pen)))
        margin = 0.10
        outside = bool(np.any(T[:3, 3] < self.box_lo - margin)
                       or np.any(T[:3, 3] > self.box_hi + margin))
        return ori_gap > 0.35 or outside

    def _takeover_approach(self, duration=2.5):
        """One smooth trajectory to the plane-ready pose before following
        starts; streaming ticks stay suspended while it plays."""
        if self.traj_client is None:
            return
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = list(self.joint_names)
        pt = JointTrajectoryPoint()
        pt.positions = self.q_ready.tolist()
        pt.velocities = [0.0] * len(self.joint_names)
        pt.time_from_start = rospy.Duration(duration)
        goal.trajectory.points = [pt]
        goal.trajectory.header.stamp = rospy.Time.now()
        self.traj_client.send_goal(goal)
        self.q_cmd = self.q_ready.copy()
        self.p_target = _fk(self.q_cmd)[:3, 3].copy()
        self._q_sent = None
        self._desired_filt = None
        self._height_filtered_m = None
        self._cart_velocity = np.zeros(3)
        self._cart_acceleration = np.zeros(3)
        self._qd_cmd = np.zeros(7)
        self._stream_hold_until = rospy.get_time() + duration + 0.5
        rospy.logwarn('Teleop take-over from a gesture pose — moving to the '
                      'ready pose (%.1fs) before following starts.', duration)

    def _publish_active(self):
        pub = getattr(self, '_active_pub', None)
        if pub is not None:
            pub.publish(Bool(data=bool(self.enabled)))

    def _set_enabled(self, enabled, source):
        if enabled and not self.enabled:
            # Resync the command state to where the arm ACTUALLY is: without
            # this, the first streamed goal would command a jump from the real
            # pose to the stale q_cmd within command_time (reflex on the real
            # arm). From the resynced pose, max_joint_step ramps toward the
            # cursor target at a bounded speed.
            if self.latest_joints is not None:
                self.q_cmd = self.latest_joints.copy()
                self.p_target = _fk(self.q_cmd)[:3, 3].copy()
                self._desired_filt = None
                self._height_filtered_m = None
                self._cart_velocity = np.zeros(3)
                self._cart_acceleration = np.zeros(3)
                self._qd_cmd = np.zeros(7)
                self._stream_hold_until = 0.0
            elif not self.dry_run:
                rospy.logwarn('Cannot enable teleop (%s): no joint state received '
                              'on %s yet — is the arm running?',
                              source, self.joint_state_topic)
                return
            if self.traj_client is not None:
                # Stop any running gesture trajectory before taking over.
                self.traj_client.cancel_all_goals()
            self._lift_paused = False
            rospy.logwarn('TELEOP ENABLED (%s) — the arm now follows the cursor.', source)
            if not self.dry_run and self._needs_takeover_approach():
                # Wait a beat so the cancel above has landed at the controller
                # (same race as home-on-exit), then run the approach.
                time.sleep(0.3)
                self._takeover_approach()
        elif not enabled and self.enabled:
            # Order matters: stop the streaming ticks FIRST (enabled=False),
            # then halt/cancel, then wait a beat so the empty-trajectory stop
            # and the async cancel have landed at the controller — otherwise
            # they arrive late and kill the homing goal we send next.
            self.enabled = False
            self._halt_streaming()
            if self.traj_client is not None:
                self.traj_client.cancel_all_goals()
            self._lift_paused = False
            if self.home_on_exit:
                time.sleep(0.3)
                self._go_home()
            else:
                rospy.loginfo('Teleop disabled (%s) — arm holds position.', source)
        self.enabled = bool(enabled)
        self._publish_active()

    def _on_joint_state(self, msg):
        by_name = dict(zip(msg.name, msg.position))
        try:
            self.latest_joints = np.array([by_name[n] for n in self.joint_names])
        except KeyError:
            pass  # e.g. gripper-only joint state message

    def _on_enable(self, msg):
        self._set_enabled(msg.data, '/colmag/draw_enable')

    def _on_command(self, msg):
        """Mode switching from the touchpad UI buttons (via /colmag/command)."""
        cmd = msg.data
        if cmd == 'robot:teleop':
            self._set_enabled(True, 'Teleop button')
        elif cmd in ('letter_detection', 'number_detection', 'symbol_detection'):
            self._set_enabled(False, cmd)
        elif cmd.startswith('gripper:'):
            self._handle_gripper(cmd.split(':', 1)[1])
        elif cmd in ('rotate:hold:cw', 'rotate:hold:ccw'):
            self._rot_dir = 1 if cmd.endswith(':cw') else -1
            self._rot_deadline = rospy.get_time() + 0.4
        elif cmd == 'rotate:stop':
            self._rot_dir = 0
        elif cmd == 'rotate:reference':
            self._magnet_twist_base_yaw = self.pen_yaw
            rospy.loginfo('Magnet twist reference captured at %+.1f deg.',
                          np.degrees(self.pen_yaw))
        elif cmd.startswith('rotate:set:'):
            try:
                relative_deg = float(cmd.split(':', 2)[2])
            except (IndexError, ValueError):
                rospy.logwarn('Invalid magnet twist target "%s".', cmd)
            else:
                target = (self._magnet_twist_base_yaw
                          + np.radians(relative_deg))
                self._set_pen_yaw(target, quiet=True)
        elif cmd in ('rotate:cw', 'rotate:ccw'):
            # Discrete single-step variant (kept for gesture/scripted use).
            self._rotate_pen(self.rotate_step if cmd == 'rotate:cw' else -self.rotate_step)
        elif cmd in ('plane:toggle', 'layer:next'):
            idx = self.layer_modes.index(self.plane_mode)
            self._set_layer_mode(self.layer_modes[(idx + 1) % len(self.layer_modes)], cmd)
        elif (cmd.startswith('plane:') or cmd.startswith('layer:')):
            new_mode = cmd.split(':', 1)[1]
            self._set_layer_mode(new_mode, cmd)

    def _set_layer_mode(self, new_mode, source='command'):
        if new_mode not in self.layer_modes:
            rospy.logwarn('Unknown teleop layer "%s" from %s', new_mode, source)
            return
        if new_mode == self.plane_mode:
            return

        self.plane_mode = new_mode
        if new_mode == 'horizontal':
            held = 'height z=%.2f m' % self.p_target[2]
        elif new_mode == 'vertical':
            held = 'depth x=%.2f m' % self.p_target[0]
        else:
            # Rotate layer: remember where the cursor was when entering, so the
            # wrist does not jump just because the layer changed.
            self._rotate_layer_ref_yaw = self.pen_yaw
            if self.latest_pose is not None:
                self._rotate_layer_ref_u, _ = self._cursor_uv(self.latest_pose)
            else:
                self._rotate_layer_ref_u = 0.0
            held = 'position x=%.2f y=%.2f z=%.2f m' % tuple(self.p_target)

        rospy.loginfo('Teleop layer -> %s (holding %s)', new_mode, held)

    def _set_pen_yaw(self, yaw, quiet=False):
        """Set the held tool yaw about its own axis (local z)."""
        requested = float(yaw)
        new_yaw = float(np.clip(requested, -self.max_pen_yaw, self.max_pen_yaw))
        if abs(new_yaw - self.pen_yaw) < 1e-6:
            if not quiet and abs(requested - new_yaw) > 1e-6:
                rospy.logwarn_throttle(2.0, 'Gripper rotation limit reached (%.0f deg).',
                                       np.degrees(self.pen_yaw))
            return
        self.pen_yaw = new_yaw
        c, s = np.cos(new_yaw), np.sin(new_yaw)
        rz = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        self.R_pen = self.R_pen0 @ rz
        if not quiet:
            rospy.loginfo_throttle(0.5, 'Gripper rotation: %+.0f deg', np.degrees(new_yaw))

    def _rotate_pen(self, delta, quiet=False):
        """Rotate the held tool orientation about its own axis (local z)."""
        self._set_pen_yaw(self.pen_yaw + delta, quiet=quiet)

    def _handle_gripper(self, action):
        if action == 'toggle':
            action = 'open' if self._grip_closed else 'close'
        if action not in ('open', 'close'):
            rospy.logwarn('Unknown gripper action "%s"', action)
            return
        if self.dry_run:
            rospy.loginfo('[dry] gripper %s', action)
            self._grip_closed = action == 'close'
            self._publish_grip_state()
            return
        clients = self._setup_gripper_clients()
        if clients is None:
            return
        move, grasp, stop = clients
        if action == 'close':
            goal = GraspGoal()
            goal.width = 0.0
            goal.epsilon.inner = 0.08
            goal.epsilon.outer = 0.08
            goal.speed = 0.08
            goal.force = self.grasp_force
            grasp.send_goal(goal)   # closes until contact, holds with force
        else:
            # A grasp keeps applying force until explicitly STOPPED — a plain
            # move while grasping is ignored (real arm and franka_gazebo alike).
            # Release the grasp first, then open.
            stop.send_goal(StopGoal())
            stop.wait_for_result(rospy.Duration(1.0))
            goal = MoveGoal()
            goal.width = 0.078
            goal.speed = 0.08
            move.send_goal(goal)
        self._grip_closed = action == 'close'
        self._publish_grip_state()
        rospy.loginfo('Gripper: %s', action)

    def _publish_grip_state(self):
        pub = getattr(self, '_grip_pub', None)
        if pub is not None:
            pub.publish(Bool(data=bool(self._grip_closed)))

    def _setup_gripper_clients(self):
        if self._grip_clients is not None:
            return self._grip_clients
        if not _GRIPPER_AVAILABLE:
            rospy.logwarn('franka_gripper messages not importable — gripper disabled.')
            return None
        move = actionlib.SimpleActionClient('/franka_gripper/move', MoveAction)
        grasp = actionlib.SimpleActionClient('/franka_gripper/grasp', GraspAction)
        stop = actionlib.SimpleActionClient('/franka_gripper/stop', StopAction)
        if not (move.wait_for_server(rospy.Duration(2.0))
                and grasp.wait_for_server(rospy.Duration(2.0))
                and stop.wait_for_server(rospy.Duration(2.0))):
            rospy.logwarn('franka_gripper action servers not available — launch the '
                          'arm with the gripper (fr3.launch use_gripper:=true / '
                          'fr3_real.launch load_gripper:=true).')
            return None
        self._grip_clients = (move, grasp, stop)
        return self._grip_clients

    def _on_pose(self, msg):
        self.latest_pose = msg.pose.position

    def _cursor_uv(self, pos):
        """Return cursor coordinates normalized to [-1, 1]."""
        u = np.clip(pos.x / self.input_extent, -1.0, 1.0)
        v = np.clip(pos.y / self.input_extent, -1.0, 1.0)
        return float(u), float(v)

    def _robot_height(self, raw_height_m):
        """Suppress sensor-height noise before the nonlinear robot mapping."""
        measured = float(np.clip(
            abs(float(raw_height_m)),
            self.height_sensor_min,
            self.height_sensor_max))
        if self._height_filtered_m is None:
            self._height_filtered_m = measured
            return measured

        error = measured - self._height_filtered_m
        if abs(error) <= self.height_noise_deadband:
            return self._height_filtered_m

        # Remove the noise band from real motion so crossing it does not create
        # a discontinuous target step.
        error -= np.copysign(self.height_noise_deadband, error)
        tau = self._height_tau(self._height_filtered_m)
        if tau <= 1e-9:
            alpha = 1.0
        else:
            dt = 1.0 / self.update_rate
            alpha = dt / (tau + dt)
        self._height_filtered_m += alpha * error
        return self._height_filtered_m

    def _height_tau(self, height_m):
        """Distance-adaptive low-pass time constant for the measured height.

        Interpolates linearly between the near and far time constants over the
        sensor range: the farther the magnet is from the board (where the
        1/r^3 field makes the estimate noisiest), the heavier the smoothing.
        """
        span = self.height_sensor_max - self.height_sensor_min
        if span <= 1e-9:
            return self.height_smoothing_tau
        fraction = (height_m - self.height_sensor_min) / span
        fraction = min(1.0, max(0.0, fraction))
        return (self.height_smoothing_tau
                + fraction * (self.height_smoothing_tau_far
                              - self.height_smoothing_tau))

    def _lift_gate_paused(self, pos):
        """Pause following while the magnet/cursor is lifted away from the board."""
        if self.lift_gate_z <= 0.0:
            return False
        try:
            z = abs(float(pos.z))
        except (TypeError, ValueError):
            return False
        if not np.isfinite(z):
            return False

        if self._lift_paused:
            if z <= self.lift_reengage_z:
                self._lift_paused = False
                self._height_filtered_m = float(np.clip(
                    z, self.height_sensor_min, self.height_sensor_max))
                self._desired_filt = None
                self._cart_velocity = np.zeros(3)
                self._cart_acceleration = np.zeros(3)
                self._qd_cmd = np.zeros(7)
                if self.latest_joints is not None:
                    self.q_cmd = self.latest_joints.copy()
                    self.p_target = _fk(self.q_cmd)[:3, 3].copy()
                    self._q_sent = None
                rospy.loginfo('Lift gate resumed: |pose.z| %.1f cm <= %.1f cm.',
                              100.0 * z, 100.0 * self.lift_reengage_z)
                return False
            return True

        if z > self.lift_gate_z:
            self._lift_paused = True
            self._cart_velocity = np.zeros(3)
            self._cart_acceleration = np.zeros(3)
            self._qd_cmd = np.zeros(7)
            self._halt_streaming()
            rospy.loginfo('Lift gate paused: |pose.z| %.1f cm > %.1f cm; arm holds.',
                          100.0 * z, 100.0 * self.lift_gate_z)
            return True
        return False

    def _update_rotate_layer_from_cursor(self, pos):
        """In rotate layer, cursor x turns the wrist while position holds."""
        if self.plane_mode != 'rotate':
            return
        u, _ = self._cursor_uv(pos)
        if self._rotate_layer_ref_u is None:
            self._rotate_layer_ref_u = u
            self._rotate_layer_ref_yaw = self.pen_yaw
        yaw = self._rotate_layer_ref_yaw + (u - self._rotate_layer_ref_u) * self.max_pen_yaw
        self._set_pen_yaw(yaw, quiet=True)

    def _desired_point(self, pos):
        """Absolute cursor-mapped point in the workspace box.

        The active layer controls only part of the command; everything else
        keeps the tracked target's current value.
        """
        u, v = self._cursor_uv(pos)
        target = self.p_target.copy()
        if self.plane_mode == 'horizontal':
            # Tabletop as seen from IN FRONT of the robot: cursor up -> the far
            # edge of the plane (toward the robot base, -x, which appears "top"
            # from that viewpoint), cursor right -> viewer's right (+y).
            target[0] = self.center[0] - self.sign_h * v * self.half_d
            target[1] = self.center[1] + self.sign_w * u * self.half_w
            # Flange height from magnet height, shared with the UI gauge.
            if self.height_from_magnet:
                target[2] = map_magnet_height(
                    self._robot_height(pos.z),
                    self.height_sensor_min,
                    self.height_sensor_max,
                    self.height_ee_min,
                    self.height_ee_max,
                    self.height_gain)
        elif self.plane_mode == 'vertical':
            # Upright canvas: cursor up -> higher (+z), cursor right -> +y.
            target[1] = self.center[1] + self.sign_w * u * self.half_w
            target[2] = self.center[2] + self.sign_h * v * self.half_h
        else:
            # Rotate layer: position holds; _update_rotate_layer_from_cursor()
            # maps cursor x to gripper yaw before IK runs.
            pass
        return np.clip(target, self.box_lo, self.box_hi)

    def _on_tick(self, _event):
        if self.latest_pose is None or not self.enabled:
            return

        # Take-over approach still playing: let it finish undisturbed.
        if rospy.get_time() < self._stream_hold_until:
            return

        if self._lift_gate_paused(self.latest_pose):
            return

        # Smooth hold-to-rotate: integrate a constant angular rate while the
        # keyboard keep-alive is fresh (watchdog stops it if repeats cease
        # without a release event, e.g. focus loss).
        if self._rot_dir != 0:
            if rospy.get_time() > self._rot_deadline:
                self._rot_dir = 0
                rospy.loginfo('Gripper rotation: %+.0f deg', np.degrees(self.pen_yaw))
            else:
                self._rotate_pen(self._rot_dir * self.rotate_speed / self.update_rate,
                                 quiet=True)
        self._update_rotate_layer_from_cursor(self.latest_pose)

        # Smooth the measured destination, then advance the position/velocity/
        # acceleration state along a bounded minimum-jerk quintic spline.
        desired = self._desired_point(self.latest_pose)
        if self.target_tau > 0.0:
            dt = 1.0 / self.update_rate
            alpha = dt / (self.target_tau + dt)
            if self._desired_filt is None:
                self._desired_filt = desired.copy()
            self._desired_filt += alpha * (desired - self._desired_filt)
            desired = self._desired_filt
        target, next_cart_velocity, next_cart_acceleration = _s_curve_step(
            self.p_target,
            self._cart_velocity,
            self._cart_acceleration,
            desired,
            1.0 / self.update_rate,
            self.s_curve_min_duration,
            self.max_lin_speed,
            self.max_lin_acceleration,
            self.max_lin_jerk)
        target = np.clip(target, self.box_lo, self.box_hi)

        # Warm-seeded tracking solves converge in a few iterations; a tight
        # cap keeps the worst-case tick inside the 33 ms budget (measured:
        # residuals are identical at caps 10/15/60, but uncapped stall-solves
        # took 20-40 ms and made the streaming loop itself miss ticks).
        q_new, err = _solve_ik(target, self.R_pen, self.q_cmd, iters=15)

        if err > self.max_reach_error:
            # Do not commit p_target: hold at the last reachable point so the
            # arm stops at the boundary instead of drifting through a dead zone.
            rospy.logwarn_throttle(2.0, 'Target %s unreachable (IK residual %.1f mm '
                                   '> %.1f mm) — holding.',
                                   np.round(target, 3), err * 1000,
                                   self.max_reach_error * 1000)
            return
        self.p_target = target
        self._cart_velocity = next_cart_velocity
        self._cart_acceleration = next_cart_acceleration

        # Clamp per-joint change so no single update can command a big jump.
        step = np.clip(q_new - self.q_cmd, -self.max_joint_step, self.max_joint_step)
        self.q_cmd = self.q_cmd + step

        # Velocity tag for the trajectory point: the exact slope of the
        # commanded sequence. The controller fits a cubic between its current
        # desired state and (position, velocity) at the deadline, so any
        # mismatch between the two makes every 100 ms segment bulge around
        # the true path. The sequence is already smooth — the S-curve bounds
        # acceleration and jerk upstream — so its raw slope is smooth too;
        # low-pass filtering it here would only add lag, i.e. mismatch.
        self._qd_cmd = step * self.update_rate

        if self.dry_run:
            rospy.loginfo_throttle(0.5, '[dry] target %s | IK err %.2f mm',
                                   np.round(target, 3), err * 1000)
            return

        if self.traj_pub is None:
            self._setup_trajectory_client()
            if self.traj_pub is None:
                return

        # Converged (cursor still, arm on target): send nothing. The controller
        # holds position on its own, and idle re-sends only churn its queue.
        if self._q_sent is not None and float(np.max(np.abs(self.q_cmd - self._q_sent))) < 1e-6:
            return

        traj = JointTrajectory()
        traj.joint_names = list(self.joint_names)
        pt = JointTrajectoryPoint()
        pt.positions = self.q_cmd.tolist()
        pt.velocities = self._qd_cmd.tolist()
        pt.time_from_start = rospy.Duration(self.command_time)
        traj.points = [pt]

        # Look-ahead point one command_time further along the spline. If the
        # next tick is late (Python GC, a slow IK solve), the controller keeps
        # moving along the prediction instead of finishing the segment and
        # snapping its desired velocity to zero — a jolt per late tick.
        p2, _, _ = _s_curve_step(
            self.p_target, self._cart_velocity, self._cart_acceleration,
            desired, self.command_time, self.s_curve_min_duration,
            self.max_lin_speed, self.max_lin_acceleration, self.max_lin_jerk)
        p2 = np.clip(p2, self.box_lo, self.box_hi)
        q2, err2 = _solve_ik(p2, self.R_pen, self.q_cmd, iters=15)
        if err2 <= self.max_reach_error:
            lookahead_ticks = self.command_time * self.update_rate
            step2 = np.clip(q2 - self.q_cmd,
                            -self.max_joint_step * lookahead_ticks,
                            self.max_joint_step * lookahead_ticks)
            pt2 = JointTrajectoryPoint()
            pt2.positions = (self.q_cmd + step2).tolist()
            pt2.velocities = (step2 / self.command_time).tolist()
            pt2.time_from_start = rospy.Duration(2.0 * self.command_time)
            traj.points.append(pt2)

        traj.header.stamp = self._next_stream_stamp(rospy.Time.now())
        self.traj_pub.publish(traj)
        self._q_sent = self.q_cmd.copy()

    def run(self):
        rospy.spin()


def main():
    try:
        ColmagDrawNode().run()
    except rospy.ROSInterruptException:
        pass
    return 0


if __name__ == '__main__':
    sys.exit(main())
