#!/usr/bin/env python3
"""
colmag_sensor_node.py — raw sensor publisher for the COLMAG gesture interface.

This is the ONLY node that touches the serial port.  All other nodes subscribe
to the topics published here instead of reading from hardware directly.

Topics published:
  /colmag/sensor_data  (std_msgs/Float64MultiArray)  — 48 Bx/By/Bz floats + 6 pose floats
  /colmag/pose         (geometry_msgs/PoseStamped)   — magnet XYZ position

Usage (inside Docker after catkin build):
  roslaunch colmag_ros colmag_distributed.launch
  # or directly:
  rosrun colmag_ros colmag_sensor_node.py _port:=/dev/ttyUSB0 _baudrate:=921600
"""

import struct
import threading
import time

import rospy
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    serial = None

PACKET_HEADER = 0xAA
PACKET_TAIL   = 0xBB
PACKET_SIZE   = 218        # 1 header + 216 data (54 floats × 4 bytes) + 1 tail
PACKET_STRUCT = struct.Struct('<54f')
BAUD_DEFAULT  = 921600


def find_serial_port():
    if serial is None:
        return None
    ports = [p.device for p in serial.tools.list_ports.comports()]
    return ports[0] if ports else None


class ColmagSensorNode:
    def __init__(self):
        rospy.init_node('colmag_sensor_node', anonymous=False)

        self.port     = rospy.get_param('~port', '')
        self.baudrate = rospy.get_param('~baudrate', BAUD_DEFAULT)
        self.verbose  = rospy.get_param('~verbose', False)

        if not self.port:
            self.port = find_serial_port() or ''
        if not self.port:
            rospy.logwarn(
                'No serial port specified and none auto-detected. '
                'Set _port:=/dev/ttyUSBx  or  _port:=/dev/ttyACMx'
            )

        self.pub_sensor = rospy.Publisher(
            '/colmag/sensor_data', Float64MultiArray, queue_size=5
        )
        self.pub_pose = rospy.Publisher(
            '/colmag/pose', PoseStamped, queue_size=5
        )

        self.serial_port = None
        self.buf = bytearray()

        rospy.loginfo(
            f'colmag_sensor_node ready | port={self.port} | baud={self.baudrate}'
        )

    # ── Serial helpers ─────────────────────────────────────────────────────────

    def open_port(self):
        if serial is None:
            rospy.logerr('pyserial not installed — cannot open serial port')
            return False
        try:
            self.serial_port = serial.Serial(
                self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1.0,
            )
            self.serial_port.reset_input_buffer()
            rospy.loginfo(f'Opened {self.port} at {self.baudrate} baud')
            return True
        except Exception as exc:
            rospy.logerr(f'Could not open {self.port}: {exc}')
            return False

    def find_next_packet(self):
        while True:
            try:
                start = self.buf.index(PACKET_HEADER)
            except ValueError:
                self.buf.clear()
                return None
            if len(self.buf) - start < PACKET_SIZE:
                if start:
                    del self.buf[:start]
                return None
            candidate = self.buf[start:start + PACKET_SIZE]
            if candidate[-1] == PACKET_TAIL:
                del self.buf[:start + PACKET_SIZE]
                return bytes(candidate)
            del self.buf[:start + 1]

    def parse_packet(self, data):
        if len(data) != PACKET_SIZE:
            return None
        if data[0] != PACKET_HEADER or data[-1] != PACKET_TAIL:
            return None
        try:
            floats = PACKET_STRUCT.unpack(data[1:217])
        except struct.error:
            return None
        return floats[:48], floats[48:]   # mag_data, pose_data

    # ── Publishing ─────────────────────────────────────────────────────────────

    def publish(self, mag_data, pose_data):
        now = rospy.Time.now()

        msg = Float64MultiArray()
        msg.data = list(mag_data) + list(pose_data)
        self.pub_sensor.publish(msg)

        pose_msg = PoseStamped()
        pose_msg.header.stamp    = now
        pose_msg.header.frame_id = 'colmag_base'
        pose_msg.pose.position.x = float(pose_data[0])
        pose_msg.pose.position.y = float(pose_data[1])
        pose_msg.pose.position.z = float(pose_data[2])
        pose_msg.pose.orientation.w = 1.0
        self.pub_pose.publish(pose_msg)

        if self.verbose:
            x, y, z = pose_data[:3]
            rospy.logdebug(f'pose  x={x:.4f}  y={y:.4f}  z={z:.4f}')

    # ── Main loop ──────────────────────────────────────────────────────────────

    def run(self):
        if not self.port or not self.open_port():
            rospy.logwarn('No serial port — sensor node is idle (spinning).')
            rospy.spin()
            return

        rospy.loginfo('Reading sensor packets…')
        rate = rospy.Rate(200)

        while not rospy.is_shutdown():
            try:
                waiting = self.serial_port.in_waiting
                if waiting:
                    self.buf.extend(self.serial_port.read(waiting))
                    while True:
                        raw = self.find_next_packet()
                        if raw is None:
                            break
                        parsed = self.parse_packet(raw)
                        if parsed:
                            self.publish(*parsed)
            except Exception as exc:
                rospy.logerr(f'Serial read error: {exc}')
                time.sleep(1.0)
            rate.sleep()

        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()


def main():
    node = ColmagSensorNode()
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
