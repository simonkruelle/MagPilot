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
        # Velocity shaping. "trapezoid" samples a ramp-cruise-ramp profile so
        # each leg starts and ends at rest; "linear" is the original 3-waypoint,
        # positions-only behaviour (constant velocity per leg, stepped reversal).
        self.velocity_profile = str(rospy.get_param("~velocity_profile", "trapezoid")).lower()
        self.ramp_fraction = float(rospy.get_param("~ramp_fraction", 0.3))
        self.sample_dt = float(rospy.get_param("~sample_dt", 0.05))

        if not 1 <= self.joint_index <= 7:
            raise ValueError("~joint_index must be in 1..7")
        if self.velocity_profile not in ("trapezoid", "linear"):
            raise ValueError("~velocity_profile must be 'trapezoid' or 'linear'")
        if not 0.0 < self.ramp_fraction <= 0.5:
            raise ValueError("~ramp_fraction must be in (0, 0.5]")
        if self.sample_dt <= 0.0:
            raise ValueError("~sample_dt must be > 0")

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

    def _trapezoid_leg(self, base, moving_start, moving_end, t0, dur):
        """Sample a trapezoidal-velocity profile for one leg on self.joint_index.

        Velocity ramps linearly 0 -> v_peak over ramp_fraction*dur, cruises at
        v_peak, then ramps back to 0, so the leg begins and ends at rest and the
        area under v(t) equals the commanded displacement. Each sample carries
        both position and velocity, which makes joint_trajectory_controller
        interpolate cubically between the dense samples.
        """
        idx = self.joint_index - 1
        distance = moving_end - moving_start
        ramp = min(max(self.ramp_fraction, 1e-3), 0.5)
        t_ramp = ramp * dur
        v_peak = distance / (dur * (1.0 - ramp)) if dur > 0 else 0.0
        accel = v_peak / t_ramp if t_ramp > 0 else 0.0

        n = max(1, int(round(dur / self.sample_dt)))
        points = []
        for k in range(1, n + 1):
            t = dur * k / n
            if t < t_ramp:
                vel = accel * t
                offset = 0.5 * accel * t * t
            elif t <= dur - t_ramp:
                vel = v_peak
                offset = 0.5 * accel * t_ramp * t_ramp + v_peak * (t - t_ramp)
            else:
                t_left = dur - t
                vel = accel * t_left
                offset = distance - 0.5 * accel * t_left * t_left

            positions = list(base)
            positions[idx] = moving_start + offset
            velocities = [0.0] * len(base)
            velocities[idx] = vel

            point = JointTrajectoryPoint()
            point.positions = positions
            point.velocities = velocities
            point.time_from_start = rospy.Duration(t0 + t)
            points.append(point)

        # Snap the final sample exactly onto the endpoint at rest.
        points[-1].positions[idx] = moving_end
        points[-1].velocities[idx] = 0.0
        return points

    def _linear_points(self, start, target):
        """Original 3-waypoint, positions-only profile (constant velocity/leg)."""
        start_point = JointTrajectoryPoint()
        start_point.positions = list(start)
        start_point.time_from_start = rospy.Duration(self.start_delay)

        target_point = JointTrajectoryPoint()
        target_point.positions = target
        target_point.time_from_start = rospy.Duration(self.move_time)

        return_point = JointTrajectoryPoint()
        return_point.positions = list(start)
        return_point.time_from_start = rospy.Duration(self.return_time)

        return [start_point, target_point, return_point]

    def _trajectory_goal(self, start):
        idx = self.joint_index - 1
        target = list(start)
        target[idx] += self.delta

        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = list(self.joint_names)

        if self.velocity_profile == "linear":
            goal.trajectory.points = self._linear_points(start, target)
            return goal, target

        # Trapezoidal (default): hold briefly at the start pose, then ramp out to
        # the target and back, each leg starting and ending at rest so the mid
        # reversal has no velocity step.
        hold = JointTrajectoryPoint()
        hold.positions = list(start)
        hold.velocities = [0.0] * len(start)
        hold.time_from_start = rospy.Duration(self.start_delay)

        out_leg = self._trapezoid_leg(
            start, start[idx], target[idx], self.start_delay, self.move_time - self.start_delay
        )
        back_leg = self._trapezoid_leg(
            start, target[idx], start[idx], self.move_time, self.return_time - self.move_time
        )

        goal.trajectory.points = [hold] + out_leg + back_leg
        return goal, target

    def _cancel_on_shutdown(self):
        client = getattr(self, "_client", None)
        if client is None:
            return
        try:
            client.cancel_all_goals()
            rospy.logwarn("Shutdown: cancelled trajectory — the arm stops and holds position.")
        except Exception as exc:
            rospy.logwarn("Shutdown: could not cancel trajectory: %s", exc)

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

        # On Ctrl-C, cancel the goal so the controller does not keep playing the
        # rest of the trajectory; the arm stops and holds position.
        self._client = client
        rospy.on_shutdown(self._cancel_on_shutdown)

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
