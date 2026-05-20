#!/usr/bin/env python3
"""
colmag_listener.py — test subscriber for the COLMAG gesture interface.

Listens to /colmag/command and /colmag/classifier and prints what it receives.
Run this inside the Docker container to verify the full pipeline works.

Usage:
    rosrun colmag_ros colmag_listener.py
"""

import rospy
from std_msgs.msg import String, Float64

last_label = None
last_confidence = 0.0


def on_classifier(msg):
    global last_label, last_confidence
    last_label = msg.data


def on_confidence(msg):
    global last_confidence
    last_confidence = msg.data


def on_command(msg):
    cmd = msg.data
    if cmd == 'choice:0':
        label = last_label or '??'
        conf = last_confidence * 100
        rospy.loginfo(f'*** CONFIRMED: "{label}" ({conf:.1f}%) — robot would execute this command ***')
    elif cmd == 'letter_detection':
        rospy.loginfo('Mode switched: letters OCR')
    elif cmd == 'number_detection':
        rospy.loginfo('Mode switched: digits OCR')
    elif cmd == 'canvas:reset':
        rospy.loginfo('Canvas reset')
    elif cmd.startswith('choice:'):
        rospy.loginfo(f'Button {cmd} pressed (runner-up selection)')
    else:
        rospy.loginfo(f'Command: {cmd}')


rospy.init_node('colmag_listener')
rospy.Subscriber('/colmag/classifier', String, on_classifier)
rospy.Subscriber('/colmag/confidence', Float64, on_confidence)
rospy.Subscriber('/colmag/command',    String, on_command)

rospy.loginfo('colmag_listener ready — draw a letter and dwell on button 1 to confirm')
rospy.spin()
