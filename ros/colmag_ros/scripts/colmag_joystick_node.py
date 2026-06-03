#!/usr/bin/env python3
"""
colmag_joystick_node.py — virtual joystick state machine for the COLMAG interface.

Subscribes to the magnet pose and classifier results.  Runs the virtual
joystick dwell-detection logic and publishes a command string whenever the
user confirms a button (e.g. holds the magnet over button 1 for 1.5 s).

Swap this node to change the interaction model (e.g. swipe gestures instead
of dwell) without touching the sensor or classifier nodes.

Topics subscribed:
  /colmag/pose        (geometry_msgs/PoseStamped) — magnet XY position
  /colmag/classifier  (std_msgs/String)            — latest predicted label
  /colmag/confidence  (std_msgs/Float64)           — latest confidence

Topics published:
  /colmag/command  (std_msgs/String)
    Values:
      'letter_detection'    — user activated letter OCR mode
      'number_detection'    — user activated digit OCR mode
      'symbol_detection'    — user activated future signs/shape mode
      'canvas:reset'        — canvas cleared
      'choice:0' … 'choice:3' — user confirmed candidate 1-4

Parameters:
  ~dwell_seconds       (float, default 1.5)   — dwell time to press a button
  ~projection_extent   (float, default 0.05)  — half-width of the pose plane (m)
  ~letter_labels       (str,   default 'ABCDEFGHIJKLMNOPQRSTUVWXYZ')
  ~digit_labels        (str,   default '0123456789')

Usage:
  rosrun colmag_ros colmag_joystick_node.py
"""

import sys
import time

import rospy
from std_msgs.msg import String, Float64
from geometry_msgs.msg import PoseStamped

sys.path.insert(0, '/colmag')   # project root mounted inside Docker

try:
    from colmag.interaction import AppController, VirtualJoystick
    _COLMAG_AVAILABLE = True
except ImportError:
    _COLMAG_AVAILABLE = False
    rospy.logwarn(
        'colmag.interaction not importable — '
        'make sure /colmag is mounted (check docker_setup.sh)'
    )


class ColmagJoystickNode:
    def __init__(self):
        rospy.init_node('colmag_joystick_node', anonymous=False)

        # ── Parameters ────────────────────────────────────────────────────────
        dwell_seconds  = float(rospy.get_param('~dwell_seconds',     1.5))
        extent         = float(rospy.get_param('~projection_extent', 0.05))
        letter_labels  = str(rospy.get_param('~letter_labels',     'ABCDEFGHIJKLMNOPQRSTUVWXYZ'))
        digit_labels   = str(rospy.get_param('~digit_labels',      '0123456789'))

        # ── Virtual joystick state machine ────────────────────────────────────
        if _COLMAG_AVAILABLE:
            joystick = VirtualJoystick.default(extent)
            self.controller = AppController(
                joystick,
                dwell_seconds=dwell_seconds,
                letter_labels=tuple(letter_labels),
                digit_labels=tuple(digit_labels),
            )
        else:
            self.controller = None
            rospy.logwarn('AppController unavailable — joystick node is passive')

        # ── Classifier state (tracked to include in log messages) ─────────────
        self.last_label      = None
        self.last_confidence = 0.0

        # ── Publishers ────────────────────────────────────────────────────────
        self.pub_cmd = rospy.Publisher('/colmag/command', String, queue_size=10)

        # ── Subscribers ───────────────────────────────────────────────────────
        rospy.Subscriber('/colmag/pose',       PoseStamped, self.on_pose)
        rospy.Subscriber('/colmag/classifier', String,      self.on_classifier)
        rospy.Subscriber('/colmag/confidence', Float64,     self.on_confidence)

        rospy.loginfo(
            f'colmag_joystick_node ready | '
            f'dwell={dwell_seconds}s | '
            f'letters={letter_labels} | '
            f'digits={digit_labels}'
        )

    # ── Classifier state tracking ──────────────────────────────────────────────

    def on_classifier(self, msg):
        self.last_label = msg.data

    def on_confidence(self, msg):
        self.last_confidence = msg.data

    # ── Main logic: pose → button dwell → command ─────────────────────────────

    def on_pose(self, msg):
        if self.controller is None:
            return

        x   = msg.pose.position.x
        y   = msg.pose.position.y
        now = time.monotonic()

        event = self.controller.update_cursor(x, y, now)
        if event is None:
            return

        cmd = event.command
        self.pub_cmd.publish(String(data=cmd))

        # Human-readable log message
        if cmd == 'choice:0' and self.last_label:
            rospy.loginfo(
                f'/colmag/command → "{cmd}" '
                f'(confirmed: {self.last_label}  {self.last_confidence * 100:.1f}%)'
            )
        else:
            rospy.loginfo(f'/colmag/command → "{cmd}"')

    def run(self):
        rospy.spin()


def main():
    node = ColmagJoystickNode()
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
