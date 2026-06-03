#!/usr/bin/env python3
"""
colmag_robot_node.py — robot action executor for the COLMAG gesture interface.

This is the terminal node of the pipeline.  It receives confirmed commands
from the joystick node and executes the corresponding robot action.

THIS IS THE NODE YOU SHOULD MODIFY to integrate with your robot.
Replace the execute_command() method with your actual robot API calls
(e.g. ROS action clients, MoveIt, joint trajectory publishers, HTTP calls).

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
  ~robot_ip     (str,   default '')     — placeholder for your robot's IP
  ~dry_run      (bool,  default true)   — if true, log commands but do not move

Usage:
  rosrun colmag_ros colmag_robot_node.py
  # With a real robot:
  rosrun colmag_ros colmag_robot_node.py _robot_ip:=192.168.1.100 _dry_run:=false
"""

import rospy
from std_msgs.msg import String, Float64


class ColmagRobotNode:
    def __init__(self):
        rospy.init_node('colmag_robot_node', anonymous=False)

        # ── Parameters ────────────────────────────────────────────────────────
        self.robot_ip = rospy.get_param('~robot_ip', '')
        self.dry_run  = rospy.get_param('~dry_run',  True)

        # ── Classifier state ──────────────────────────────────────────────────
        self.last_label      = None
        self.last_confidence = 0.0

        # ── Subscribers ───────────────────────────────────────────────────────
        rospy.Subscriber('/colmag/command',    String,  self.on_command)
        rospy.Subscriber('/colmag/classifier', String,  self.on_classifier)
        rospy.Subscriber('/colmag/confidence', Float64, self.on_confidence)

        mode = 'DRY RUN (no robot actions)' if self.dry_run else f'LIVE (robot_ip={self.robot_ip})'
        rospy.loginfo(f'colmag_robot_node ready | mode={mode}')

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
                f'CONFIRMED gesture: "{label}" ({conf:.1f}%) — '
                f'{"(dry run)" if self.dry_run else "executing"}'
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
            rospy.loginfo(f'Runner-up choice {idx} selected')
        else:
            rospy.loginfo(f'Command received: {cmd}')

    def execute_command(self, label: str):
        """
        Execute a robot action for the confirmed character label.

        REPLACE THIS METHOD with your robot API calls.

        Examples:
          - Send a joint trajectory for a waving motion on 'W'
          - Call a MoveIt pick-and-place action on 'P'
          - Publish a velocity command on 'F' (forward) / 'B' (back)
          - Send an HTTP request to a web-based robot controller

        Args:
            label: The confirmed character, e.g. 'A', '3', 'X'
        """
        rospy.loginfo(f'execute_command("{label}") — TODO: add your robot action here')

        # ── Example skeleton (uncomment and adapt) ────────────────────────────
        # if label == 'F':
        #     self.move_forward()
        # elif label == 'B':
        #     self.move_backward()
        # elif label == 'L':
        #     self.turn_left()
        # elif label == 'R':
        #     self.turn_right()
        # else:
        #     rospy.logwarn(f'No action mapped for label "{label}"')

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
