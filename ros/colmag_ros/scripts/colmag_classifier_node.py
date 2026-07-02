#!/usr/bin/env python3
"""
colmag_classifier_node.py — OCR character classifier for the COLMAG gesture interface.

Subscribes to the magnet pose stream, accumulates a canvas image from pose
samples that pass the velocity gate, and runs EasyOCR after the user stops
writing.  Swap this node to use a custom trained model without touching
anything else.

Topics subscribed:
  /colmag/pose     (geometry_msgs/PoseStamped)  — magnet position from sensor_node
  /colmag/command  (std_msgs/String)             — listens for 'canvas:reset'

Topics published:
  /colmag/classifier  (std_msgs/String)   — top predicted character (e.g. "A")
  /colmag/confidence  (std_msgs/Float64)  — prediction confidence 0..1

Parameters (set via _param:=value or in launch file):
  ~image_size          (int,   default 64)    — canvas resolution in pixels
  ~projection_extent   (float, default 0.05)  — half-width of the pose plane in metres
  ~idle_seconds        (float, default 0.8)   — writing pause before OCR fires
  ~min_velocity        (float, default 0.04)  — minimum XY velocity to count as ink
  ~min_ink_samples     (int,   default 8)     — minimum ink points before OCR runs
  ~writing_max_z       (float, default -1)    — suppress ink when |z| > this (metres);
                                                 -1 means disabled

Usage:
  rosrun colmag_ros colmag_classifier_node.py
"""

import sys
import threading
import time
from collections import deque

import numpy as np
import rospy
from std_msgs.msg import String, Float64
from geometry_msgs.msg import PoseStamped

sys.path.insert(0, '/colmag')   # project root mounted inside Docker


class ColmagClassifierNode:
    def __init__(self):
        rospy.init_node('colmag_classifier_node', anonymous=False)

        # ── Parameters ────────────────────────────────────────────────────────
        self.image_size      = rospy.get_param('~image_size',        64)
        self.extent          = rospy.get_param('~projection_extent', 0.05)
        self.idle_seconds    = rospy.get_param('~idle_seconds',      0.8)
        self.min_velocity    = rospy.get_param('~min_velocity',      0.04)
        self.min_ink_samples = rospy.get_param('~min_ink_samples',   8)
        self.writing_max_z   = rospy.get_param('~writing_max_z',     -1.0)

        # ── Canvas state ──────────────────────────────────────────────────────
        self.canvas       = np.ones((self.image_size, self.image_size), dtype=np.float32)
        self.canvas_lock  = threading.Lock()
        self.last_pose    = None   # (x, y, z, t) of previous sample
        self.ink_count    = 0
        self.last_write_t = 0.0
        self.writing      = False  # True while velocity >= min_velocity
        self.last_submitted_ink = -1

        # Pre-compute Gaussian brush kernel once
        # (matches the touchpad UI brush: slightly thinner than 1 px/64)
        radius = max(0.6, self.image_size / 64.0 * 0.75)
        bext   = int(np.ceil(radius * 2.5))
        y_ker, x_ker = np.ogrid[-bext:bext + 1, -bext:bext + 1]
        self._brush_kernel = np.exp(
            -(x_ker ** 2 + y_ker ** 2) / (2.0 * radius ** 2)
        )
        self._brush_radius = radius
        self._brush_extent = bext

        # ── Classifier (EasyOCR — optional) ───────────────────────────────────
        self.classifier = None
        self._load_classifier()

        # ── ROS publishers ────────────────────────────────────────────────────
        self.pub_label = rospy.Publisher('/colmag/classifier', String,  queue_size=5)
        self.pub_conf  = rospy.Publisher('/colmag/confidence', Float64, queue_size=5)

        # ── ROS subscribers ───────────────────────────────────────────────────
        rospy.Subscriber('/colmag/pose',    PoseStamped, self.on_pose)
        rospy.Subscriber('/colmag/command', String,      self.on_command)

        # ── Timer: fire OCR after idle period ─────────────────────────────────
        rospy.Timer(rospy.Duration(0.1), self.on_timer)

        rospy.loginfo(
            f'colmag_classifier_node ready | '
            f'image={self.image_size}px | '
            f'idle={self.idle_seconds}s | '
            f'min_vel={self.min_velocity} | '
            f'classifier={"EasyOCR" if self.classifier else "none (install easyocr)"}'
        )

    # ── Classifier loading ─────────────────────────────────────────────────────

    def _load_classifier(self):
        try:
            import easyocr
            # gpu=False keeps this working inside a CPU-only Docker container
            self.classifier = easyocr.Reader(['en'], gpu=False)
            rospy.loginfo('EasyOCR loaded successfully')
        except ImportError:
            rospy.logwarn(
                'EasyOCR not installed — classification is disabled. '
                'Rebuild Docker with: INSTALL_EASYOCR=1 bash ros/docker_setup.sh'
            )
        except Exception as exc:
            rospy.logwarn(f'EasyOCR failed to load: {exc}')

    # ── Pose callback ──────────────────────────────────────────────────────────

    def on_pose(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z
        t = msg.header.stamp.to_sec()
        if t == 0.0:
            t = time.time()

        with self.canvas_lock:
            if self.last_pose is not None:
                px, py, pz, pt = self.last_pose
                dt = t - pt
                if dt > 0:
                    vel = np.sqrt((x - px) ** 2 + (y - py) ** 2) / dt
                    z_ok = (
                        self.writing_max_z < 0
                        or abs(z) <= self.writing_max_z
                    )
                    if vel >= self.min_velocity and z_ok:
                        self._paint(x, y)
                        self.last_write_t = time.monotonic()
                        self.writing = True
                    else:
                        self.writing = False
            self.last_pose = (x, y, z, t)

    def _paint(self, x, y):
        size = self.image_size
        ext  = self.extent
        bext = self._brush_extent

        px = int((x / ext + 1.0) / 2.0 * (size - 1))
        py = int((1.0 - (y / ext + 1.0) / 2.0) * (size - 1))
        px = max(bext, min(size - 1 - bext, px))
        py = max(bext, min(size - 1 - bext, py))

        self.canvas[py - bext:py + bext + 1, px - bext:px + bext + 1] = np.minimum(
            self.canvas[py - bext:py + bext + 1, px - bext:px + bext + 1],
            1.0 - self._brush_kernel,
        )
        self.ink_count += 1

    # ── Command callback (canvas/mode reset) ─────────────────────────────────

    def on_command(self, msg):
        if msg.data in ('canvas:reset', 'letter_detection', 'number_detection', 'symbol_detection'):
            with self.canvas_lock:
                self.canvas[:] = 1.0
                self.ink_count = 0
                self.last_submitted_ink = -1
                self.writing = False
            rospy.loginfo(f'Canvas reset for command: {msg.data}')

    # ── Timer: idle detection → classify ──────────────────────────────────────

    def on_timer(self, _event):
        image = None
        now   = time.monotonic()

        with self.canvas_lock:
            if (
                not self.writing
                and self.ink_count >= self.min_ink_samples
                and self.ink_count != self.last_submitted_ink
                and self.last_write_t > 0
                and now - self.last_write_t >= self.idle_seconds
            ):
                self.last_submitted_ink = self.ink_count
                image = self.canvas.copy()

        if image is not None:
            self._run_classifier(image)

    # ── OCR inference ──────────────────────────────────────────────────────────

    def _run_classifier(self, image):
        """
        Run EasyOCR on the canvas snapshot.
        Replace this method with your own model call if you train a custom classifier.
        The method must publish to /colmag/classifier (String) and /colmag/confidence (Float64).
        """
        if self.classifier is None:
            return

        try:
            import PIL.Image
            img_u8 = (image * 255.0).clip(0, 255).astype(np.uint8)
            pil    = PIL.Image.fromarray(img_u8).resize((128, 128), PIL.Image.LANCZOS)
            results = self.classifier.readtext(
                np.array(pil),
                allowlist='ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789',
            )
            if not results:
                return

            # EasyOCR returns list of (bbox, text, confidence)
            best  = max(results, key=lambda r: r[2])
            label = best[1].strip().upper()
            conf  = float(best[2])

            if label:
                self.pub_label.publish(String(data=label))
                self.pub_conf.publish(Float64(data=conf))
                rospy.loginfo(f'/colmag/classifier → "{label}" ({conf * 100:.1f}%)')

        except Exception as exc:
            rospy.logwarn(f'Classifier inference error: {exc}')

    def run(self):
        rospy.spin()


def main():
    node = ColmagClassifierNode()
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
