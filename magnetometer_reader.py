#!/usr/bin/env python3
# PYTHON_ARGCOMPLETE_OK
"""
Magnetometer Data Reader for Collaborative Robotics Seminar
Reads serial data from magnetometer sensor and processes it.
"""

import threading
import time
import struct
import csv
import os
import json
import re

os.environ.setdefault('XDG_CACHE_HOME', '/private/tmp/colmag-cache')
os.environ.setdefault('MPLCONFIGDIR', '/private/tmp/colmag-matplotlib-cache')

import matplotlib.pyplot as plt
from matplotlib import patches
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import signal
import sys
import argcomplete
from collections import deque
import argparse
from datetime import datetime
import select

from colmag.interaction import AppController, InputMode, VirtualJoystick
from colmag.magnetic_sim import (
    apply_magnetic_calibration,
    load_magnetic_calibration,
    synthetic_dipole_values,
)

try:
    import serial
    import serial.tools.list_ports
except ImportError as exc:
    serial = None
    SERIAL_IMPORT_ERROR = exc
else:
    SERIAL_IMPORT_ERROR = None

# ROS 1 publishing support — optional, gracefully absent when ROS is not installed.
try:
    import rospy
    from std_msgs.msg import String, Float64, Float64MultiArray
    from geometry_msgs.msg import PoseStamped
    _ROS_AVAILABLE = True
except ImportError:
    _ROS_AVAILABLE = False


class _RosBridge:
    """Thin wrapper that publishes colmag events to ROS 1 topics."""

    TOPICS = {
        'sensor_data': ('/colmag/sensor_data', 'Float64MultiArray', 5),
        'pose':        ('/colmag/pose',         'PoseStamped',       5),
        'command':     ('/colmag/command',      'String',            10),
        'classifier':  ('/colmag/classifier',   'String',            5),
        'confidence':  ('/colmag/confidence',   'Float64',           5),
    }

    def __init__(self, node_name='colmag_node'):
        if not _ROS_AVAILABLE:
            raise RuntimeError('rospy not available — install ROS Noetic inside the Docker container')
        rospy.init_node(node_name, anonymous=False, disable_signals=True)
        self._pub_sensor = rospy.Publisher('/colmag/sensor_data', Float64MultiArray, queue_size=5)
        self._pub_pose   = rospy.Publisher('/colmag/pose',         PoseStamped,        queue_size=5)
        self._pub_cmd    = rospy.Publisher('/colmag/command',      String,             queue_size=10)
        self._pub_label  = rospy.Publisher('/colmag/classifier',   String,             queue_size=5)
        self._pub_conf   = rospy.Publisher('/colmag/confidence',   Float64,            queue_size=5)
        print('[ROS] colmag_node registered with ROS Master')
        print('[ROS] Topics: /colmag/{sensor_data, pose, command, classifier, confidence}')

    def publish_sensor(self, mag_data, pose_data):
        msg = Float64MultiArray()
        msg.data = list(mag_data) + list(pose_data)
        self._pub_sensor.publish(msg)

        pose_msg = PoseStamped()
        pose_msg.header.stamp    = rospy.Time.now()
        pose_msg.header.frame_id = 'colmag_base'
        pose_msg.pose.position.x = float(pose_data[0])
        pose_msg.pose.position.y = float(pose_data[1])
        pose_msg.pose.position.z = float(pose_data[2])
        pose_msg.pose.orientation.w = 1.0
        self._pub_pose.publish(pose_msg)

    def publish_command(self, command):
        self._pub_cmd.publish(String(data=command))
        print(f'[ROS] /colmag/command → "{command}"')

    def publish_classifier(self, label, confidence):
        self._pub_label.publish(String(data=label))
        self._pub_conf.publish(Float64(data=confidence))

SERIAL_EXCEPTION = serial.SerialException if serial is not None else Exception

PACKET_SIZE = 218
PACKET_HEADER = 0xAA
PACKET_TAIL = 0xBB
POSE_LIMIT = 0.05
DEFAULT_CLASSIFIER_LABELS = tuple('0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ')
CLASSIFIER_LABEL_PRESETS = {
    'digits': tuple('0123456789'),
    'letters': tuple('ABCDEFGHIJKLMNOPQRSTUVWXYZ'),
    'alphanumeric': DEFAULT_CLASSIFIER_LABELS,
}
DATA_MANIFEST_SCHEMA_VERSION = 1

RECORDING_TARGETS = (
    tuple(f'digit_{d}' for d in '0123456789') +
    tuple(f'letter_{c}' for c in 'ABCDEFGHIJKLMNOPQRSTUVWXYZ') +
    ('control_blank', 'control_still')
)
TARGET_REPS_DEFAULT = 5


def top_label_indices(scores, count):
    """Return label indices sorted by descending score, keeping label order for ties."""
    ranked = list(range(len(scores)))
    ranked.sort(key=lambda index: (-float(scores[index]), index))
    return ranked[:count]


def display_labels_for(labels):
    if isinstance(labels, str):
        return CLASSIFIER_LABEL_PRESETS.get(labels, tuple(labels.upper()))
    return tuple(labels)


class MagnetometerReader:
    def __init__(
        self,
        plot_sensor=1,
        verbose=False,
        trail_length=250,
        image_size=64,
        projection_extent=POSE_LIMIT,
        z_close_mode='max',
        z_near=None,
        z_far=None,
        image_output_dir='digit_images',
        enable_classifier=True,
        classifier_gpu=False,
        classifier_labels='alphanumeric',
        lazy_classifier=True,
        classifier_cpu_threads=1,
        enable_writing_filter=True,
        writing_min_velocity=0.01,
        writing_max_velocity=None,
        writing_min_closeness=None,
        joystick_dwell_seconds=1.5,
        letter_labels='letters',
        digit_labels='digits',
        classifier_interval=0.75,
        classifier_mode='on-idle',
        classifier_idle_seconds=0.8,
        classifier_min_ink_samples=8,
        input_source='serial',
        touchpad_z=0.02,
        touchpad_sample_rate=100.0,
        touchpad_ink_strength=1.0,
        touchpad_synthetic_magnetics=True,
        touchpad_magnetic_scale=1e-6,
        touchpad_ink_mode='velocity',
        touchpad_magnetic_calibration=None,
        touchpad_speed_log=None,
        touchpad_speed_report_interval=1.0,
        display_window=2000,
        plot_fps=None,
        # Modes
        record_data=False,
        dry_run=False,
        validation_mode=False,
        output_dir=None,
        run_id=None,
        # ROS bridge
        ros=False,
    ):
        self.serial_port = None
        self.is_running = False
        self.read_thread = None
        self.data_lock = threading.Lock()
        self.csv_file = None
        self.csv_writer = None
        self.plot_sensor = plot_sensor  # Which sensor to plot (1-16)
        self.verbose = verbose
        self.packet_struct = struct.Struct('<54f')
        self.packet_count = 0
        self.bad_packet_count = 0
        self.trail_length = trail_length
        self.image_size = image_size
        self.projection_extent = projection_extent
        self.z_close_mode = z_close_mode
        self.z_near = z_near
        self.z_far = z_far
        self.image_output_dir = image_output_dir
        self.prediction_history = deque(maxlen=5)
        self.classifier = None
        self.enable_classifier = enable_classifier
        self.classifier_gpu = classifier_gpu
        self.classifier_labels = classifier_labels
        self.classifier_requested_labels = classifier_labels
        self.lazy_classifier = lazy_classifier
        self.classifier_cpu_threads = classifier_cpu_threads
        self.classifier_load_error = None
        self.classifier_loading = False
        self.classifier_load_thread = None
        self._classifier_frame_skip = 0
        self.classifier_interval = classifier_interval
        self.classifier_mode = classifier_mode
        self.classifier_idle_seconds = classifier_idle_seconds
        self.classifier_min_ink_samples = classifier_min_ink_samples
        if touchpad_sample_rate <= 0:
            raise ValueError("touchpad_sample_rate must be positive")
        if not 0.0 <= touchpad_ink_strength <= 1.0:
            raise ValueError("touchpad_ink_strength must be between 0 and 1")
        if touchpad_ink_mode not in ('pen', 'velocity'):
            raise ValueError("touchpad_ink_mode must be 'pen' or 'velocity'")
        self.input_source = input_source
        self.touchpad_z = touchpad_z
        self.touchpad_sample_rate = float(touchpad_sample_rate)
        self.touchpad_ink_strength = float(touchpad_ink_strength)
        self.touchpad_synthetic_magnetics = touchpad_synthetic_magnetics
        self.touchpad_magnetic_scale = float(touchpad_magnetic_scale)
        self.touchpad_ink_mode = touchpad_ink_mode
        self.touchpad_magnetic_calibration_file = touchpad_magnetic_calibration
        self.touchpad_magnetic_calibration = None
        if touchpad_magnetic_calibration:
            try:
                self.touchpad_magnetic_calibration = load_magnetic_calibration(
                    touchpad_magnetic_calibration
                )
                print(f"Loaded touchpad magnetic calibration: {touchpad_magnetic_calibration}")
            except Exception as e:
                raise ValueError(f"Could not load touchpad magnetic calibration: {e}") from e
        self.touchpad_speed_log = touchpad_speed_log
        self.touchpad_speed_report_interval = float(touchpad_speed_report_interval)
        self.touchpad_sample_interval = 1.0 / self.touchpad_sample_rate
        self.display_window = display_window
        self.plot_fps = plot_fps
        self.record_data = record_data
        self.dry_run = dry_run
        self.validation_mode = validation_mode
        self.output_dir = output_dir
        self.run_id = self.sanitize_path_component(run_id) if run_id else run_id
        self.command_line = list(sys.argv)
        self.raw_csv_path = None
        self.raw_csv_enabled = False
        self.manifest_path = (
            os.path.join(self.output_dir, 'manifest.json')
            if self.output_dir
            else None
        )
        buffer_length = 1000
        if self.input_source == 'touchpad':
            buffer_length = max(buffer_length, int(self.touchpad_sample_rate * 300))
        self.data_buffer = deque(maxlen=buffer_length)
        self.touchpad_state = {
            'x': 0.0,
            'y': 0.0,
            'inside': True,
            'pen_down': False,
            'last_sample_at': 0.0,
            'last_sample_x': 0.0,
            'last_sample_y': 0.0,
            'last_sample_pen_down': False,
        }
        self.touchpad_speed_samples = deque(maxlen=5000)
        self.touchpad_last_speed = 0.0
        self.touchpad_last_speed_report_at = 0.0
        self.touchpad_speed_log_file = None
        self.touchpad_speed_log_writer = None
        self.live_digit_image = np.ones((self.image_size, self.image_size), dtype=float)
        self.live_digit_processed_count = 0
        self.live_digit_last_timestamp = None
        self.live_ink_count = 0
        self.live_ink_count = 0
        self.classifier_candidates = []
        self.action_feedback_text = ""
        self.action_feedback_button = None
        self.action_feedback_until = 0.0
        self.touchpad_pen_enabled = True
        self.ros_bridge = _RosBridge() if ros else None
        self.classifier_thread = None
        self.classifier_stop_event = threading.Event()
        self.classifier_input_event = threading.Event()
        self.classifier_state_lock = threading.Lock()
        self.classifier_inference_lock = threading.Lock()
        self.classifier_pending_image = None
        self.classifier_latest_result = None
        self.classifier_latest_error = None
        self.classifier_status_text = "OCR idle"
        self.classifier_last_submit_time = 0.0
        self.classifier_last_writing_time = 0.0
        self.classifier_waiting_for_pause = False
        self.classifier_last_submitted_ink_count = -1
        self.classification_started_at = None
        self.latest_prediction_text = "Prediction: --"
        self.latest_runner_up_text = "Runner-up: --"
        self.enable_writing_filter = enable_writing_filter
        self.writing_min_velocity = writing_min_velocity
        self.writing_max_velocity = writing_max_velocity
        self.writing_min_closeness = writing_min_closeness
        self.joystick = VirtualJoystick.default(self.projection_extent)
        self.app_controller = AppController(
            self.joystick,
            dwell_seconds=joystick_dwell_seconds,
            letter_labels=letter_labels,
            digit_labels=digit_labels,
        )
        self.latest_interface_text = "Mode: menu | Button: -- | Last: --"

        if enable_classifier and not lazy_classifier:
            self.load_classifier()
        elif enable_classifier:
            self.latest_prediction_text = self.classifier_unloaded_prediction_text()

        # Fix 5: Pre-compute Gaussian brush kernel once (avoids np.ogrid + np.exp per point per frame)
        self._brush_radius = max(0.75, self.image_size / 64.0)
        self._brush_extent = int(np.ceil(self._brush_radius * 2.5))
        y_ker, x_ker = np.ogrid[
            -self._brush_extent:self._brush_extent + 1,
            -self._brush_extent:self._brush_extent + 1
        ]
        self._brush_kernel = np.exp(
            -(x_ker * x_ker + y_ker * y_ker) / (2.0 * self._brush_radius ** 2)
        )

        # Session recording
        self.recording_sessions = []  # List of (session_name, start_time, end_time, data_points)
        self.current_session = None
        self.session_data = []
        self.current_session_started_at = None

        # Setup signal handler for graceful shutdown
        signal.signal(signal.SIGINT, self.signal_handler)

    @staticmethod
    def csv_header():
        """Return the shared row header for raw and per-session CSV files."""
        header = ['timestamp']
        for i in range(16):
            header.extend([f'Sensor{i+1}_Bx', f'Sensor{i+1}_By', f'Sensor{i+1}_Bz'])
        header.extend(['Pose_x', 'Pose_y', 'Pose_z', 'Pose_mx', 'Pose_my', 'Pose_mz'])
        return header

    @staticmethod
    def sanitize_path_component(value):
        """Make a user/run label safe as a single filename component."""
        safe = re.sub(r'[^A-Za-z0-9_.-]+', '_', str(value).strip())
        safe = safe.strip('._-')
        return safe or 'unnamed'

    def session_label(self, session_name):
        """Return the exact label folder name for a recording session."""
        return self.sanitize_path_component(session_name)

    def path_relative_to_output(self, path):
        """Return a manifest-friendly path relative to output_dir when possible."""
        if not path:
            return None
        if not self.output_dir:
            return path
        try:
            return os.path.relpath(path, self.output_dir)
        except ValueError:
            return path

    def collect_git_metadata(self):
        """Collect git branch/commit/dirty metadata for reproducibility."""
        git_info = {}
        try:
            import subprocess
            commands = {
                'commit': ['git', 'log', '-1', '--format=%H', '--no-show-signature'],
                'branch': ['git', 'branch', '--show-current'],
                'status': ['git', 'status', '--porcelain'],
            }
            for key, command in commands.items():
                result = subprocess.run(
                    command,
                    capture_output=True,
                    text=True,
                    timeout=3,
                )
                if result.returncode == 0:
                    if key == 'status':
                        git_info['dirty'] = len(result.stdout.strip()) > 0
                    else:
                        git_info[key] = result.stdout.strip()
        except Exception:
            git_info['error'] = 'git metadata unavailable'
        return git_info

    def manifest_settings(self):
        """Return the settings block stored in run and session metadata."""
        return {
            'image_size': self.image_size,
            'trail_length': self.trail_length,
            'projection_extent': self.projection_extent,
            'z_close_mode': self.z_close_mode,
            'z_near': self.z_near,
            'z_far': self.z_far,
            'classifier_labels': ''.join(display_labels_for(self.classifier_labels)),
            'classifier_mode': self.classifier_mode,
            'classifier_interval': self.classifier_interval,
            'classifier_idle_seconds': self.classifier_idle_seconds,
            'classifier_min_ink_samples': self.classifier_min_ink_samples,
            'writing_filter': {
                'enabled': self.enable_writing_filter,
                'min_velocity': self.writing_min_velocity,
                'max_velocity': self.writing_max_velocity,
                'min_closeness': self.writing_min_closeness,
            },
            'touchpad': {
                'z': self.touchpad_z,
                'sample_rate': self.touchpad_sample_rate,
                'ink_strength': self.touchpad_ink_strength,
                'ink_mode': self.touchpad_ink_mode,
                'synthetic_magnetics': self.touchpad_synthetic_magnetics,
                'magnetic_scale': self.touchpad_magnetic_scale,
                'magnetic_calibration_file': self.touchpad_magnetic_calibration_file,
                'magnetic_calibration_model': (
                    self.touchpad_magnetic_calibration.get('model')
                    if self.touchpad_magnetic_calibration
                    else None
                ),
            },
            'display_window': self.display_window,
            'plot_fps': self.plot_fps,
        }

    def prepare_recording_layout(self, raw_csv_path=None, raw_csv_enabled=None):
        """Create the record/dry-run folder layout and write the run manifest."""
        if not self.output_dir:
            raise ValueError("output_dir is required for recording layout")
        if not self.run_id:
            self.run_id = datetime.now().strftime('run_%Y%m%d_%H%M%S')
        self.run_id = self.sanitize_path_component(self.run_id)
        self.manifest_path = os.path.join(self.output_dir, 'manifest.json')
        if raw_csv_path is not None:
            self.raw_csv_path = raw_csv_path
        if raw_csv_enabled is not None:
            self.raw_csv_enabled = bool(raw_csv_enabled)

        os.makedirs(self.output_dir, exist_ok=True)
        os.makedirs(os.path.join(self.output_dir, 'raw'), exist_ok=True)
        os.makedirs(os.path.join(self.output_dir, 'samples'), exist_ok=True)
        self.write_manifest()

    def load_manifest(self):
        """Load the current manifest, returning a minimal base if absent."""
        if not self.manifest_path or not os.path.exists(self.manifest_path):
            return {
                'schema_version': DATA_MANIFEST_SCHEMA_VERSION,
                'created_at': datetime.now().isoformat(),
                'sessions': [],
            }

        try:
            with open(self.manifest_path, 'r', encoding='utf-8') as f:
                manifest = json.load(f)
        except (IOError, json.JSONDecodeError):
            manifest = {
                'schema_version': DATA_MANIFEST_SCHEMA_VERSION,
                'created_at': datetime.now().isoformat(),
                'sessions': [],
            }

        manifest.setdefault('schema_version', DATA_MANIFEST_SCHEMA_VERSION)
        manifest.setdefault('created_at', datetime.now().isoformat())
        manifest.setdefault('sessions', [])
        return manifest

    def write_manifest(self, session_entry=None):
        """Write the manifest atomically, optionally appending one session."""
        if not self.manifest_path:
            return

        manifest = self.load_manifest()
        sessions = manifest.get('sessions', [])
        if session_entry is not None:
            sessions.append(session_entry)

        manifest.update({
            'schema_version': DATA_MANIFEST_SCHEMA_VERSION,
            'updated_at': datetime.now().isoformat(),
            'run_id': self.run_id,
            'output_dir': os.path.abspath(self.output_dir) if self.output_dir else None,
            'command_line': self.command_line,
            'input_source': self.input_source,
            'record_data': self.record_data,
            'dry_run': self.dry_run,
            'validation_mode': self.validation_mode,
            'git': self.collect_git_metadata(),
            'settings': self.manifest_settings(),
            'raw_log': {
                'enabled': bool(self.raw_csv_enabled and self.raw_csv_path),
                'path': self.path_relative_to_output(self.raw_csv_path),
            },
            'sessions': sessions,
        })

        temp_path = f"{self.manifest_path}.tmp"
        with open(temp_path, 'w', encoding='utf-8') as f:
            json.dump(manifest, f, indent=2)
            f.write('\n')
        os.replace(temp_path, self.manifest_path)

    def next_session_repetition(self, label):
        """Return the next repetition number for run_id + exact label."""
        next_rep = 1
        manifest = self.load_manifest()
        for session in manifest.get('sessions', []):
            if session.get('run_id') == self.run_id and session.get('label') == label:
                try:
                    next_rep = max(next_rep, int(session.get('repetition', 0)) + 1)
                except (TypeError, ValueError):
                    next_rep = max(next_rep, 2)

        label_dir = os.path.join(self.output_dir, 'samples', label)
        basename_prefix = f"{self.run_id}_{label}_rep"
        if os.path.isdir(label_dir):
            for filename in os.listdir(label_dir):
                match = re.match(
                    rf"{re.escape(basename_prefix)}(\d+)_.*\.(?:csv|png|json)$",
                    filename,
                )
                if match:
                    next_rep = max(next_rep, int(match.group(1)) + 1)
        return next_rep

    def reps_by_label(self):
        """Count completed repetitions per label for this run from in-memory session list."""
        counts = {}
        for (session_name, *_) in self.recording_sessions:
            label = self.session_label(session_name)
            counts[label] = counts.get(label, 0) + 1
        return counts

    def print_recording_checklist(self, target_reps=TARGET_REPS_DEFAULT):
        """Print an ASCII checklist showing recording progress for all standard targets."""
        counts = self.reps_by_label()

        digits  = [f'digit_{d}' for d in '0123456789']
        letters = [f'letter_{c}' for c in 'ABCDEFGHIJKLMNOPQRSTUVWXYZ']
        controls = ['control_blank', 'control_still']

        done_labels = sum(1 for lbl in RECORDING_TARGETS if counts.get(lbl, 0) >= target_reps)
        total_reps  = sum(counts.values())
        needed_reps = len(RECORDING_TARGETS) * target_reps

        def cell(label, display):
            n = counts.get(label, 0)
            mark = 'v' if n >= target_reps else ' '
            return f"{display}[{n}{mark}]"

        dig_row = ' '.join(cell(lbl, d) for lbl, d in zip(digits, '0123456789'))
        let_row1 = ' '.join(cell(lbl, c) for lbl, c in zip(letters[:13], 'ABCDEFGHIJKLM'))
        let_row2 = ' '.join(cell(lbl, c) for lbl, c in zip(letters[13:], 'NOPQRSTUVWXYZ'))
        ctl_row  = '  '.join(cell(lbl, name) for lbl, name in zip(controls, ['blank(-)', 'still(=)']))

        print()
        print("=" * 60)
        print(f"RECORDING PROGRESS  (target: {target_reps} reps each, v = done)")
        print(f"  Digits:   {dig_row}")
        print(f"  Letters:  {let_row1}")
        print(f"            {let_row2}")
        print(f"  Controls: {ctl_row}")
        print(f"  Done: {done_labels}/{len(RECORDING_TARGETS)} labels | {total_reps}/{needed_reps} reps total")
        missing = [
            lbl.replace('digit_', '').replace('letter_', '').replace('control_', '')
            for lbl in RECORDING_TARGETS if counts.get(lbl, 0) < target_reps
        ]
        if missing:
            shown = ' '.join(missing[:25])
            extra = f' (+{len(missing)-25} more)' if len(missing) > 25 else ''
            print(f"  Still needed: {shown}{extra}")
        print("=" * 60)

    def create_session_paths(self, session_name):
        """Create a matched CSV/PNG/JSON basename and return all paths."""
        label = self.session_label(session_name)
        label_dir = os.path.join(self.output_dir, 'samples', label)
        os.makedirs(label_dir, exist_ok=True)

        repetition = self.next_session_repetition(label)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        basename = f"{self.run_id}_{label}_rep{repetition:03d}_{timestamp}"
        return {
            'label': label,
            'label_dir': label_dir,
            'repetition': repetition,
            'timestamp': timestamp,
            'basename': basename,
            'csv_path': os.path.join(label_dir, f"{basename}.csv"),
            'image_path': os.path.join(label_dir, f"{basename}.png"),
            'metadata_path': os.path.join(label_dir, f"{basename}.json"),
        }

    def load_classifier(self):
        """Load the optional pretrained EasyOCR character classifier."""
        try:
            from digit_classifier.inference import DigitClassifier

            self.classifier = DigitClassifier(
                gpu=self.classifier_gpu,
                labels=self.classifier_requested_labels,
                cpu_threads=self.classifier_cpu_threads,
            )
            self.classifier_labels = self.classifier.labels
            self.classifier_requested_labels = self.classifier.labels
            self.classifier_load_error = None
            self.classifier_loading = False
            print(f"Loaded EasyOCR character classifier ({''.join(self.classifier.labels)})")
            self.start_classifier_worker()
        except Exception as e:
            self.classifier = None
            self.classifier_load_error = e
            self.classifier_loading = False
            print(f"Character classifier not loaded: {e}")

    def ensure_classifier_loading(self):
        """Kick off lazy EasyOCR loading without blocking the UI."""
        if not self.enable_classifier or self.classifier is not None or self.classifier_loading:
            return

        self.classifier_loading = True
        with self.classifier_state_lock:
            self.classifier_status_text = "OCR loading"
        self.classifier_load_thread = threading.Thread(target=self.load_classifier, daemon=True)
        self.classifier_load_thread.start()

    def start_classifier_worker(self):
        """Start the background EasyOCR worker used by the live UI."""
        if self.classifier is None:
            return
        if self.classifier_thread and self.classifier_thread.is_alive():
            return

        self.classifier_stop_event.clear()
        self.classifier_thread = threading.Thread(target=self.classifier_loop, daemon=True)
        self.classifier_thread.start()

    def classifier_loop(self):
        """Run OCR off the Matplotlib update path so the UI stays responsive."""
        current_labels = tuple(self.classifier.labels)
        history = deque(maxlen=5)

        while not self.classifier_stop_event.is_set():
            self.classifier_input_event.wait(timeout=0.1)
            if self.classifier_stop_event.is_set():
                break

            with self.classifier_state_lock:
                image = self.classifier_pending_image
                labels = self.classifier_requested_labels
                self.classifier_pending_image = None
                if image is None:
                    self.classifier_input_event.clear()
                    continue
                self.classifier_status_text = "OCR running"
                if self.classifier_pending_image is None:
                    self.classifier_input_event.clear()

            try:
                started_at = time.monotonic()
                with self.classifier_inference_lock:
                    resolved_labels = self.classifier._resolve_labels(labels)
                    if resolved_labels != current_labels:
                        self.classifier.set_labels(labels)
                        current_labels = tuple(self.classifier.labels)
                        history.clear()
                        with self.classifier_state_lock:
                            self.classifier_labels = self.classifier.labels
                        print(f"Classifier labels switched to: {''.join(self.classifier.labels)}")

                    result = self.classifier.predict_smoothed(image, history)
                elapsed = time.monotonic() - started_at
                with self.classifier_state_lock:
                    self.classifier_latest_result = result
                    self.classifier_latest_error = None
                    self.classifier_status_text = f"OCR {elapsed:.2f}s"
            except Exception as e:
                with self.classifier_state_lock:
                    self.classifier_latest_error = e
                    self.classifier_status_text = "OCR failed"

    def submit_classifier_image(self, image, force=False):
        """Queue the newest image for background OCR, throttled by interval."""
        if self.classifier is None and not self.enable_classifier:
            return

        now = time.monotonic()
        if not force and now - self.classifier_last_submit_time < self.classifier_interval:
            return

        self.classifier_last_submit_time = now
        with self.classifier_state_lock:
            self.classifier_pending_image = np.asarray(image, dtype=np.float32).copy()
            if self.classifier is None:
                self.classifier_status_text = "OCR loading"
            elif self.classifier_status_text != "OCR running":
                self.classifier_status_text = "OCR queued"
            self.classifier_input_event.set()

        if self.classifier is None:
            self.ensure_classifier_loading()

    def reset_live_classification_session(self):
        """Forget pending live OCR trigger state after mode/label changes."""
        self.classifier_last_writing_time = 0.0
        self.classifier_waiting_for_pause = False
        self.classifier_last_submitted_ink_count = -1
        self.classifier_last_submit_time = 0.0
        self.classification_started_at = None
        self.reset_live_digit_image()

    def mark_classification_start(self, timestamp):
        """Start a fresh writing capture window for the active OCR mode."""
        try:
            self.classification_started_at = datetime.fromisoformat(str(timestamp))
        except (TypeError, ValueError):
            self.classification_started_at = None
        self.classifier_last_submitted_ink_count = -1
        self.classifier_waiting_for_pause = False
        self.reset_live_digit_image()

    def reset_live_digit_image(self):
        """Clear the incremental live OCR canvas."""
        self.live_digit_image = np.ones((self.image_size, self.image_size), dtype=float)
        self.live_digit_processed_count = 0
        self.live_digit_last_timestamp = None

    def classification_sample_mask(self, rows):
        """Return which rows are part of the current writing capture window."""
        if self.classification_started_at is None:
            return np.ones(len(rows), dtype=bool)

        mask = []
        for row in rows:
            try:
                timestamp = datetime.fromisoformat(str(row[0]))
            except (TypeError, ValueError):
                mask.append(True)
            else:
                mask.append(timestamp >= self.classification_started_at)
        return np.asarray(mask, dtype=bool)

    def get_classifier_state(self):
        """Return the latest background OCR output for the UI thread."""
        with self.classifier_state_lock:
            return (
                self.classifier_latest_result,
                self.classifier_latest_error,
                self.classifier_status_text,
            )

    def classifier_unloaded_prediction_text(self):
        """Describe the classifier state before an EasyOCR reader is available."""
        if not self.enable_classifier:
            return "Prediction: classifier disabled"
        if self.classifier_loading:
            return "Prediction: classifier loading"
        if self.classifier_load_error is not None:
            return "Prediction: classifier failed to load"
        return "Prediction: classifier ready on first use"

    def set_classifier_labels(self, labels):
        """Switch the OCR allowlist without reloading EasyOCR weights."""
        self.classifier_labels = labels
        self.classifier_requested_labels = labels
        self.classifier_candidates = []
        self.prediction_history.clear()
        self.reset_live_classification_session()

        if self.classifier is not None:
            with self.classifier_state_lock:
                self.classifier_requested_labels = labels
                self.classifier_latest_result = None
                self.classifier_latest_error = None
                self.classifier_status_text = "OCR labels queued"
            self.latest_prediction_text = "Prediction: --"
            self.latest_runner_up_text = "Runner-up: --"
            print(f"Classifier labels scheduled: {''.join(display_labels_for(labels))}")
        else:
            if self.enable_classifier:
                self.latest_prediction_text = self.classifier_unloaded_prediction_text()
                self.latest_runner_up_text = "Runner-up: --"
                if self.classifier_load_error:
                    print(f"Classifier labels queued for next load: {labels}")
                    print(f"Classifier load error was: {self.classifier_load_error}")
                else:
                    print(f"Classifier labels queued for next load: {labels}")
            else:
                self.latest_prediction_text = "Prediction: classifier disabled"
                print(
                    f"Classifier labels queued for next load: {labels}. "
                    "Run without --no-classifier to show OCR predictions."
                )

    def handle_interface_event(self, event):
        """Apply a virtual joystick event to the live app state."""
        if event.classifier_labels is not None:
            self.set_classifier_labels(event.classifier_labels)
        if event.command:
            if self.ros_bridge:
                self.ros_bridge.publish_command(event.command)
            if event.command.startswith('choice:'):
                self.confirm_classifier_candidate(event.command, event.button.name)
            elif event.command == 'canvas:reset':
                self.reset_writing_canvas()
                self._set_key_feedback("Canvas reset", duration=1.0)
            elif event.command == 'letter_detection':
                self._set_key_feedback("Mode: letters")
                print(f"Virtual joystick: {event.button.name} -> {event.command}")
            elif event.command == 'number_detection':
                self._set_key_feedback("Mode: digits")
                print(f"Virtual joystick: {event.button.name} -> {event.command}")
            else:
                self._set_key_feedback(f"{event.button.name}: {event.command.split(':', 1)[-1]}")
                print(f"Virtual joystick: {event.button.name} -> {event.command}")

    def confirm_classifier_candidate(self, command, button_name):
        """Confirm one of the currently displayed classifier candidates."""
        try:
            candidate_index = int(command.split(':', 1)[1])
        except (IndexError, ValueError):
            print(f"Virtual joystick: {button_name} -> invalid candidate command")
            return

        if candidate_index >= len(self.classifier_candidates):
            print(f"Virtual joystick: {button_name} -> no classifier candidate yet")
            return

        label, confidence = self.classifier_candidates[candidate_index]
        confirmed_command = f"confirm:{label}"
        self.app_controller.last_command = confirmed_command
        self.action_feedback_text = f"Completed: {label} ({confidence * 100:.1f}%)"
        self.action_feedback_button = button_name
        self.action_feedback_until = time.monotonic() + 2.0
        print(
            f"Virtual joystick: {button_name} -> {confirmed_command} "
            f"({confidence * 100:.1f}%)"
        )

    def reset_writing_canvas(self):
        """Clear the current live writing canvas without leaving the OCR mode."""
        self.classification_started_at = datetime.now()
        self.classifier_candidates = []
        self.prediction_history.clear()
        self.classifier_last_writing_time = 0.0
        self.classifier_waiting_for_pause = False
        self.classifier_last_submitted_ink_count = -1
        self.reset_live_digit_image()
        self.action_feedback_text = ""
        self.action_feedback_button = None
        self.action_feedback_until = 0.0

        with self.classifier_state_lock:
            self.classifier_latest_result = None
            self.classifier_latest_error = None
            if self.classifier is not None:
                self.classifier_status_text = "OCR idle"

        if self.enable_classifier and self.app_controller.mode.value in ('letters', 'digits'):
            self.latest_prediction_text = "Prediction: waiting for writing"
        else:
            self.latest_prediction_text = self.classifier_unloaded_prediction_text()
        self.latest_runner_up_text = "Runner-up: --"
        print("Writing canvas reset")

    def activate_classifier_mode(self, mode_name):
        """Switch classifier modes without requiring a dwell gesture."""
        if mode_name == 'letters':
            self.app_controller.mode = InputMode.LETTERS
            self.app_controller.classifier_labels = self.app_controller.letter_labels
            self.app_controller.last_command = 'letter_detection'
            self.set_classifier_labels(self.app_controller.letter_labels)
        elif mode_name == 'digits':
            self.app_controller.mode = InputMode.DIGITS
            self.app_controller.classifier_labels = self.app_controller.digit_labels
            self.app_controller.last_command = 'number_detection'
            self.set_classifier_labels(self.app_controller.digit_labels)
        else:
            return

        rows = self.get_data_copy()
        if rows:
            self.mark_classification_start(rows[-1][0])
        print(f"Touchpad mode: {mode_name} classification active")

    def list_ports(self):
        """List all available serial ports."""
        if serial is None:
            print(
                "pyserial is not installed. Install dependencies with "
                "`pip install -r requirements.txt`."
            )
            if self.verbose:
                print(f"Import error: {SERIAL_IMPORT_ERROR}")
            return None

        ports = serial.tools.list_ports.comports()
        if not ports:
            print("No serial ports found!")
            return None

        print("Available serial ports:")
        for i, port in enumerate(ports):
            print(f"  {i+1}: {port.device} - {port.description}")
        return ports

    def choose_port(self):
        """Let user choose a serial port."""
        ports = self.list_ports()
        if not ports:
            return None

        while True:
            try:
                choice = input("Choose port number (1-{}): ".format(len(ports)))
                idx = int(choice) - 1
                if 0 <= idx < len(ports):
                    return ports[idx].device
                else:
                    print("Invalid choice. Please try again.")
            except ValueError:
                print("Please enter a number.")

    def open_port(self, port_name, baudrate=921600):
        """Open the serial port."""
        if serial is None:
            print(
                "Cannot open serial port because pyserial is not installed. "
                "Install dependencies with `pip install -r requirements.txt`."
            )
            return False

        try:
            self.serial_port = serial.Serial(
                port=port_name,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0
            )
            self.serial_port.reset_input_buffer()
            print(f"Opened serial port {port_name} at {baudrate} baud")
            return True
        except SERIAL_EXCEPTION as e:
            print(f"Error opening serial port: {e}")
            return False

    def start_csv_logging(self, filename):
        """Initialize CSV file for data logging."""
        try:
            # Create directory if it doesn't exist
            os.makedirs(os.path.dirname(filename) if os.path.dirname(filename) else '.', exist_ok=True)

            self.csv_file = open(filename, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(self.csv_header())
            self.raw_csv_path = filename
            self.raw_csv_enabled = True

            print(f"Started CSV logging to {filename}")
        except IOError as e:
            print(f"Error creating CSV file: {e}")
            return False
        return True

    def parse_packet(self, data):
        """
        Parse a complete packet.
        Expected format: 0xAA + 54 floats (216 bytes) + 0xBB

        Data format:
        - Bytes 0-215: 54 floats total
        - Floats 0-47: Magnetic field data (16 sensors × 3 axes each = 48 values)
          - Sensor 1: Bx1, By1, Bz1 (floats 0,1,2)
          - Sensor 2: Bx2, By2, Bz2 (floats 3,4,5)
          - ...
          - Sensor 16: Bx16, By16, Bz16 (floats 45,46,47)
        - Floats 48-53: Pose data (6 values)
          - x, y, z: Position coordinates
          - mx, my, mz: Rotation/orientation (likely Euler angles or quaternion components)

        Magnetic field measurement units depend on your magnetometer hardware:
        - **Tesla (T)**: Typical range 0.00001 - 0.0001 T (Earth's field ~0.00005 T)
        - **Gauss (G)**: Typical range 0.0001 - 0.001 G (1 G = 0.0001 T)
        - **microTesla (μT)**: Typical range 10 - 100 μT (Earth's field ~25-65 μT)
        - **milliGauss (mG)**: Typical range 0.1 - 1.0 mG
        """
        if len(data) != PACKET_SIZE:  # 1 header + 216 data + 1 tail
            if self.verbose:
                print(f"DEBUG - Wrong packet size: {len(data)}, expected {PACKET_SIZE}")
            return None

        if data[0] != PACKET_HEADER or data[-1] != PACKET_TAIL:
            if self.verbose:
                print(f"DEBUG - Wrong header/tail: header=0x{data[0]:02X} (expected 0xAA), tail=0x{data[-1]:02X} (expected 0xBB)")
            return None

        try:
            floats = self.packet_struct.unpack(data[1:217])  # Skip header, include 216 bytes of float data
        except struct.error as e:
            if self.verbose:
                print(f"DEBUG - Failed to unpack floats: {e}")
            return None

        # Split into magnetic field data (48 floats) and pose data (6 floats)
        mag_data = floats[:48]  # 16 sensors × 3 axes
        pose_data = floats[48:]  # 6 pose values

        return mag_data, pose_data

    def print_packet_summary(self, mag_data, pose_data):
        """Print one decoded packet. Intended for debugging, not normal real-time use."""
        print(f"\n{'='*60}")
        print(f"PACKET RECEIVED - Sensor {self.plot_sensor} (Highlighted)")
        print(f"{'='*60}")
        print("MAGNETIC FIELD DATA (Bx, By, Bz per sensor):")
        for i in range(16):
            bx = mag_data[i*3]
            by = mag_data[i*3 + 1]
            bz = mag_data[i*3 + 2]
            marker = " <-- PLOTTING" if i+1 == self.plot_sensor else ""
            print(f"  Sensor {i+1:2d}: Bx={bx:>10.6f}, By={by:>10.6f}, Bz={bz:>10.6f}{marker}")

        print("\nPOSE DATA:")
        print(f"  Position:  x={pose_data[0]:>10.6f}, y={pose_data[1]:>10.6f}, z={pose_data[2]:>10.6f}")
        print(f"  Rotation: mx={pose_data[3]:>10.6f}, my={pose_data[4]:>10.6f}, mz={pose_data[5]:>10.6f}")

    def read_serial_data(self):
        """Thread function to read serial data as fast as packets arrive."""
        buffer = bytearray()
        last_status_time = time.monotonic()
        last_status_count = 0
        last_bad_count = 0

        print("Starting serial read thread in real-time mode")

        while self.is_running:
            if self.serial_port and self.serial_port.is_open:
                try:
                    # Read available data
                    waiting = self.serial_port.in_waiting
                    data = self.serial_port.read(waiting if waiting else PACKET_SIZE)
                    if data:
                        buffer.extend(data)

                        # Look for complete packets
                        while len(buffer) >= PACKET_SIZE:
                            # Find packet start (0xAA)
                            start_idx = buffer.find(b'\xAA')
                            if start_idx == -1:
                                # No start marker found, clear buffer
                                buffer.clear()
                                break

                            # Remove data before start marker
                            if start_idx > 0:
                                del buffer[:start_idx]

                            # Check if we have a complete packet
                            if len(buffer) >= PACKET_SIZE:
                                # Check for end marker (0xBB at position 217)
                                if buffer[PACKET_SIZE - 1] == PACKET_TAIL:
                                    # Extract packet
                                    packet_data = buffer[:PACKET_SIZE]

                                    # Parse the packet
                                    parsed = self.parse_packet(packet_data)
                                    if parsed:
                                        mag_data, pose_data = parsed

                                        # Create timestamp
                                        timestamp = datetime.now().isoformat()

                                        # Create row data for CSV
                                        row_data = [timestamp]
                                        row_data.extend(mag_data)
                                        row_data.extend(pose_data)

                                        # Thread-safe data storage
                                        with self.data_lock:
                                            self.data_buffer.append(row_data)

                                            # Store in current session if recording
                                            if self.current_session:
                                                self.session_data.append(row_data)

                                        # CSV writing is kept outside the plot/session lock so disk I/O
                                        # cannot stall the UI data copy.
                                        if self.csv_writer:
                                            self.csv_writer.writerow(row_data)

                                        if self.ros_bridge:
                                            self.ros_bridge.publish_sensor(mag_data, pose_data)

                                        self.packet_count += 1

                                        if self.verbose:
                                            self.print_packet_summary(mag_data, pose_data)

                                    else:
                                        self.bad_packet_count += 1
                                        if self.verbose:
                                            print(f"DEBUG - Failed to parse packet: len={len(packet_data)}, header=0x{packet_data[0]:02X}, tail=0x{packet_data[-1]:02X}")

                                    # Remove processed packet from buffer
                                    del buffer[:PACKET_SIZE]
                                else:
                                    # End marker not found, remove start marker and continue
                                    del buffer[0]
                            else:
                                # Not enough data for complete packet, wait for more
                                break
                    else:
                        time.sleep(0.001)

                except SERIAL_EXCEPTION as e:
                    print(f"Serial read error: {e}")
                    time.sleep(0.1)

            now = time.monotonic()
            if now - last_status_time >= 1.0:
                packets_since_last = self.packet_count - last_status_count
                bad_since_last = self.bad_packet_count - last_bad_count
                total_since_last = packets_since_last + bad_since_last
                bad_ratio = (
                    bad_since_last / total_since_last
                    if total_since_last > 0
                    else 0.0
                )
                status = f"Read rate: {packets_since_last / (now - last_status_time):.1f} packets/s"
                if self.validation_mode or self.bad_packet_count:
                    status += (
                        f" | bad packets: {self.bad_packet_count} "
                        f"({bad_ratio * 100:.2f}% recent)"
                    )
                if self.current_session:
                    status += f" | RECORDING: {self.current_session} ({len(self.session_data)} points)"
                print(status)
                last_status_time = now
                last_status_count = self.packet_count
                last_bad_count = self.bad_packet_count

    def start_reading(self):
        """Start the reading thread."""
        if self.read_thread and self.read_thread.is_alive():
            print("Reading thread already running")
            return True

        self.is_running = True
        self.read_thread = threading.Thread(target=self.read_serial_data, daemon=True)
        self.read_thread.start()
        print("Started reading thread")
        return True

    def start_keyboard_thread(self):
        """Start a thread to handle keyboard input for session recording."""
        self.keyboard_thread = threading.Thread(target=self.keyboard_input_loop, daemon=True)
        self.keyboard_thread.start()

    def keyboard_input_loop(self):
        """Handle keyboard input for session recording controls."""
        print("Keyboard input thread started")

        while self.is_running:
            # Check if there's input available
            if select.select([sys.stdin], [], [], 0.1)[0]:
                try:
                    key = sys.stdin.read(1).strip()
                    self.handle_keypress(key)
                except:
                    pass

    def _set_key_feedback(self, text, duration=1.5):
        self.action_feedback_text = text
        self.action_feedback_until = time.monotonic() + duration
        self.action_feedback_button = None

    def handle_keypress(self, key):
        """Handle keyboard input for session controls."""
        if not key:
            return
        if key in '0123456789':
            self._set_key_feedback(f"REC  digit_{key}")
            self.start_session(f'digit_{key}')
        elif key == '-':
            self._set_key_feedback("REC  blank")
            self.start_session('control_blank')
        elif key == '=':
            self._set_key_feedback("REC  still")
            self.start_session('control_still')
        elif key == 's':
            self.stop_session()
        elif key == 'q':
            print("Quitting...")
            self.stop()
        elif key.isalpha() and len(key) == 1:
            self._set_key_feedback(f"REC  letter_{key.upper()}")
            self.start_session(f'letter_{key.upper()}')

    def start_session(self, session_name):
        """Start recording a new session."""
        if self.current_session:
            print(f"Already recording session '{self.current_session}'. Stop it first with 's'.")
            return

        self.current_session = session_name
        self.session_data = []
        self.current_session_started_at = datetime.now().isoformat()
        self.prediction_history.clear()
        if self.classifier is not None:
            self.latest_prediction_text = "Prediction: --"
        else:
            self.latest_prediction_text = self.classifier_unloaded_prediction_text()
        self.latest_runner_up_text = "Runner-up: --"
        label = self.session_label(session_name)
        existing_reps = self.reps_by_label().get(label, 0)
        rep_info = f"rep {existing_reps + 1}" if self.record_data else "live"
        print(f"\n>>> RECORDING: '{session_name}'  [{rep_info}]")
        print(f"    Draw — then move to classification button — then press 's' to save <<<")
        if self.record_data:
            print(f"    Save dir : {self.output_dir}/samples/{label}/")

    def stop_session(self):
        """Stop the current recording session."""
        if not self.current_session:
            print("No active recording session to stop.")
            return

        session_name = self.current_session
        data_points = len(self.session_data)

        start_time = (
            self.session_data[0][0]
            if self.session_data
            else self.current_session_started_at or datetime.now().isoformat()
        )
        end_time = self.session_data[-1][0] if self.session_data else datetime.now().isoformat()

        self.recording_sessions.append((session_name, start_time, end_time, data_points))

        label = self.session_label(session_name)
        completed_reps = self.reps_by_label().get(label, 1)

        print(f"\n<<< SAVED: '{session_name}'  rep {completed_reps}/{TARGET_REPS_DEFAULT}  ({data_points} samples) >>>")

        if self.record_data:
            self.save_session_artifacts(session_name, start_time, end_time, data_points)
            self.print_recording_checklist()
            self._set_key_feedback(
                f"Saved  {session_name}  rep {completed_reps}/{TARGET_REPS_DEFAULT}  ({data_points} pts)",
                duration=3.0,
            )
        else:
            print("(live mode — use --record-data to save CSV/PNG/JSON)")
            self._set_key_feedback(
                f"Stopped  {session_name}  ({data_points} pts, not saved)",
                duration=3.0,
            )

        self.current_session = None
        self.session_data = []
        self.current_session_started_at = None

    def save_session_artifacts(self, session_name, start_time, end_time, data_points):
        """Save matched CSV/PNG/JSON artifacts and append the manifest entry."""
        if not self.session_data:
            return

        if not self.output_dir:
            print("Cannot save session artifacts: output_dir is not configured")
            return
        if not self.run_id:
            self.run_id = datetime.now().strftime('run_%Y%m%d_%H%M%S')

        paths = self.create_session_paths(session_name)
        csv_saved = self.save_session_csv(paths['csv_path'])
        image_result = self.save_session_image(paths['image_path'])
        prediction = None
        if image_result:
            _, character_image = image_result
            prediction = self.print_final_prediction(character_image)

        metadata_path = self.save_session_metadata(
            session_name=session_name,
            paths=paths,
            start_time=start_time,
            end_time=end_time,
            data_points=data_points,
            prediction=prediction,
        )

        session_entry = {
            'session_name': session_name,
            'label': paths['label'],
            'run_id': self.run_id,
            'repetition': paths['repetition'],
            'basename': paths['basename'],
            'started_at': start_time,
            'ended_at': end_time,
            'saved_at': datetime.now().isoformat(),
            'sample_count': data_points,
            'paths': {
                'csv': self.path_relative_to_output(paths['csv_path']) if csv_saved else None,
                'png': self.path_relative_to_output(paths['image_path']) if image_result else None,
                'json': self.path_relative_to_output(metadata_path) if metadata_path else None,
            },
            'prediction': prediction,
        }
        self.write_manifest(session_entry=session_entry)
        print(f"Manifest updated: {self.manifest_path}")

    def save_session_csv(self, filename):
        """Save session data to a structured CSV file."""
        if not self.session_data:
            return False

        try:
            with open(filename, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(self.csv_header())
                for row in self.session_data:
                    writer.writerow(row)

            print(f"Session data saved to: {filename}")
            return True

        except IOError as e:
            print(f"Error saving session CSV: {e}")
            return False

    def extract_pose_trail(self, rows, trail_length=None, include_timestamps=False):
        """Return pose X/Y/Z arrays from buffered CSV-style rows."""
        if not rows:
            empty = np.array([])
            if include_timestamps:
                return empty, empty, empty, empty
            return empty, empty, empty

        limit = trail_length if trail_length is not None else self.trail_length
        rows = rows[-limit:] if limit and len(rows) > limit else rows

        pose_x = np.array([row[-6] for row in rows], dtype=float)
        pose_y = np.array([row[-5] for row in rows], dtype=float)
        pose_z = np.array([row[-4] for row in rows], dtype=float)
        if include_timestamps:
            return pose_x, pose_y, pose_z, self.rows_to_seconds(rows)
        return pose_x, pose_y, pose_z

    @staticmethod
    def rows_to_seconds(rows):
        """Return seconds from the first row timestamp, falling back to sample index."""
        if not rows:
            return np.array([])

        try:
            parsed = [datetime.fromisoformat(str(row[0])) for row in rows]
        except (TypeError, ValueError):
            return np.arange(len(rows), dtype=float)

        start = parsed[0]
        return np.array([(timestamp - start).total_seconds() for timestamp in parsed], dtype=float)

    @staticmethod
    def pose_xy_speed(pose_x, pose_y, timestamps=None):
        """Estimate per-sample XY speed from neighboring pose samples."""
        pose_x = np.asarray(pose_x, dtype=float)
        pose_y = np.asarray(pose_y, dtype=float)
        speed = np.zeros(len(pose_x), dtype=float)
        if len(pose_x) < 2:
            return speed

        if timestamps is None or len(timestamps) != len(pose_x):
            dt = np.ones(len(pose_x) - 1, dtype=float)
        else:
            dt = np.diff(np.asarray(timestamps, dtype=float))
            valid_dt = np.isfinite(dt) & (dt > 1e-6)
            fallback_dt = float(np.median(dt[valid_dt])) if np.any(valid_dt) else 1.0
            dt = np.where(valid_dt, dt, fallback_dt)

        distances = np.hypot(np.diff(pose_x), np.diff(pose_y))
        segment_speed = np.divide(distances, dt, out=np.zeros_like(distances), where=dt > 0.0)
        speed[:-1] = np.maximum(speed[:-1], segment_speed)
        speed[1:] = np.maximum(speed[1:], segment_speed)
        return speed

    def writing_sample_mask(self, pose_x, pose_y, pose_z, timestamps=None):
        """
        Identify samples that should become ink in the OCR image.

        A sample counts as writing when the magnet moves in a controlled XY
        velocity band. This rejects dots caused by holding the magnet still.
        An optional Z/closeness gate can still be enabled for board experiments,
        but air-writing keeps it off by default.
        """
        pose_x = np.asarray(pose_x, dtype=float)
        pose_y = np.asarray(pose_y, dtype=float)
        pose_z = np.asarray(pose_z, dtype=float)
        finite = np.isfinite(pose_x) & np.isfinite(pose_y) & np.isfinite(pose_z)

        speed = self.pose_xy_speed(pose_x, pose_y, timestamps)
        closeness = np.zeros(len(pose_z), dtype=float)
        if np.any(finite):
            closeness[finite] = self.z_to_closeness(pose_z[finite])

        if not self.enable_writing_filter:
            return finite, speed, closeness

        moving = speed >= self.writing_min_velocity
        if self.writing_max_velocity is not None:
            moving &= speed <= self.writing_max_velocity

        if self.writing_min_closeness is None:
            close = np.ones(len(pose_z), dtype=bool)
        else:
            close = closeness >= self.writing_min_closeness

        return finite & moving & close, speed, closeness

    def draw_pose_samples_on_image(self, image, pose_x, pose_y, pose_z, sample_mask=None):
        """Draw pose samples into an existing grayscale OCR image."""
        if len(pose_x) == 0:
            return image

        extent = self.projection_extent
        if extent <= 0:
            raise ValueError("projection_extent must be positive")

        finite = np.isfinite(pose_x) & np.isfinite(pose_y) & np.isfinite(pose_z)
        if sample_mask is not None:
            sample_mask = np.asarray(sample_mask, dtype=bool)
            if len(sample_mask) != len(pose_x):
                raise ValueError("sample_mask must have the same length as the pose arrays")
            finite &= sample_mask

        pose_x = pose_x[finite]
        pose_y = pose_y[finite]
        pose_z = pose_z[finite]
        if len(pose_x) == 0:
            return image

        x_pixels = np.rint((pose_x + extent) / (2 * extent) * (self.image_size - 1)).astype(int)
        y_pixels = np.rint((extent - pose_y) / (2 * extent) * (self.image_size - 1)).astype(int)

        valid = (
            (x_pixels >= 0) & (x_pixels < self.image_size) &
            (y_pixels >= 0) & (y_pixels < self.image_size)
        )
        if not np.any(valid):
            return image

        x_pixels = x_pixels[valid]
        y_pixels = y_pixels[valid]
        z_values = pose_z[valid]

        closeness = self.z_to_closeness(z_values)

        # Use pre-computed brush kernel (Fix 5) — avoids np.ogrid + np.exp per point
        be = self._brush_extent
        kernel = self._brush_kernel

        for x, y, close in zip(x_pixels, y_pixels, closeness):
            stroke_strength = 0.18 + 0.82 * float(np.clip(close, 0.0, 1.0))

            # Image slice for this brush
            x0 = max(0, x - be)
            x1 = min(self.image_size, x + be + 1)
            y0 = max(0, y - be)
            y1 = min(self.image_size, y + be + 1)

            # Corresponding kernel slice (handles edge clipping)
            kx0 = max(0, be - x)
            kx1 = min(2 * be + 1, be + self.image_size - x)
            ky0 = max(0, be - y)
            ky1 = min(2 * be + 1, be + self.image_size - y)

            patch = 1.0 - stroke_strength * kernel[ky0:ky1, kx0:kx1]
            image[y0:y1, x0:x1] = np.minimum(image[y0:y1, x0:x1], patch)

        return np.clip(image, 0.0, 1.0)

    def pose_to_digit_image(self, pose_x, pose_y, pose_z, sample_mask=None):
        """
        Project a 3D pose trail to a grayscale image.

        X/Y select the pixel location. Z controls stroke darkness: points closer
        to the sensor grid are darker, while untouched pixels remain white.
        """
        image = np.ones((self.image_size, self.image_size), dtype=float)
        return self.draw_pose_samples_on_image(image, pose_x, pose_y, pose_z, sample_mask)

    def live_pose_to_digit_image(self, rows, pose_x, pose_y, pose_z, sample_mask):
        """Update the live OCR canvas incrementally instead of redrawing history."""
        if len(pose_x) == 0 or not rows:
            return self.live_digit_image.copy()

        start_index = 0
        if self.live_digit_last_timestamp is not None:
            matching_indices = [
                index for index, row in enumerate(rows)
                if row and row[0] == self.live_digit_last_timestamp
            ]
            if matching_indices:
                start_index = max(0, matching_indices[-1] - 1)
            else:
                self.reset_live_digit_image()
                start_index = 0

        if len(pose_x) < self.live_digit_processed_count:
            self.reset_live_digit_image()

        if start_index < len(pose_x):
            self.live_digit_image = self.draw_pose_samples_on_image(
                self.live_digit_image,
                pose_x[start_index:],
                pose_y[start_index:],
                pose_z[start_index:],
                sample_mask[start_index:],
            )
            self.live_digit_processed_count = len(pose_x)
            self.live_digit_last_timestamp = rows[-1][0]

        return self.live_digit_image.copy()

    def update_live_digit_image_from_rows(self, rows):
        """Fast live OCR update that only processes rows unseen by the canvas."""
        pose_rows = self.display_pose_rows(rows)
        if not pose_rows:
            return self.live_digit_image.copy(), self.live_ink_count, False, None

        start_index = 0
        skip_count = 0
        if self.live_digit_last_timestamp is not None:
            matching_indices = [
                index for index, row in enumerate(pose_rows)
                if row and row[0] == self.live_digit_last_timestamp
            ]
            if matching_indices:
                last_index = matching_indices[-1]
                start_index = max(0, last_index - 1)
                skip_count = last_index - start_index + 1
            else:
                self.reset_live_digit_image()

        new_rows = pose_rows[start_index:]
        if not new_rows:
            return self.live_digit_image.copy(), self.live_ink_count, False, pose_rows[-1]

        pose_x, pose_y, pose_z, timestamps = self.extract_pose_trail(
            new_rows,
            trail_length=0,
            include_timestamps=True,
        )
        writing_mask, _, _ = self.writing_sample_mask(
            pose_x,
            pose_y,
            pose_z,
            timestamps,
        )
        if skip_count:
            writing_mask[:skip_count] = False

        pen_mask = self.touchpad_pen_mask(new_rows)
        if pen_mask is not None:
            writing_mask &= pen_mask
        writing_mask &= ~self.joystick_button_mask(pose_x, pose_y)
        writing_mask &= self.classification_sample_mask(new_rows)

        new_ink_count = int(np.count_nonzero(writing_mask))
        if new_ink_count:
            self.live_digit_image = self.draw_pose_samples_on_image(
                self.live_digit_image,
                pose_x,
                pose_y,
                pose_z,
                writing_mask,
            )
            self.live_ink_count += new_ink_count

        self.live_digit_last_timestamp = pose_rows[-1][0]
        self.live_digit_processed_count = len(pose_rows)
        return self.live_digit_image.copy(), self.live_ink_count, bool(len(writing_mask) and writing_mask[-1]), pose_rows[-1]

    @staticmethod
    def normalize_values(values):
        values = np.asarray(values, dtype=float)
        value_min = np.nanmin(values)
        value_max = np.nanmax(values)
        if not np.isfinite(value_min) or not np.isfinite(value_max) or value_max == value_min:
            return np.full_like(values, 0.5)
        return (values - value_min) / (value_max - value_min)

    def z_to_closeness(self, z_values):
        """Map Z values to 0..1 closeness, where 1 creates the darkest stroke."""
        z_values = np.asarray(z_values, dtype=float)

        if self.input_source == 'touchpad' and self.z_near is None and self.z_far is None:
            return np.full_like(z_values, np.clip(self.touchpad_ink_strength, 0.0, 1.0))

        if self.z_near is not None and self.z_far is not None:
            denominator = self.z_near - self.z_far
            if denominator == 0:
                return np.full_like(z_values, 0.5)
            return np.clip((z_values - self.z_far) / denominator, 0.0, 1.0)

        if self.z_close_mode == 'max':
            return self.normalize_values(z_values)
        if self.z_close_mode == 'abs-min':
            return 1.0 - self.normalize_values(np.abs(z_values))
        return 1.0 - self.normalize_values(z_values)

    def rows_to_digit_image(self, rows, trail_length=None):
        pose_x, pose_y, pose_z, timestamps = self.extract_pose_trail(
            rows,
            trail_length=trail_length,
            include_timestamps=True,
        )
        writing_mask, _, _ = self.writing_sample_mask(pose_x, pose_y, pose_z, timestamps)
        pen_mask = self.touchpad_pen_mask(rows[-len(writing_mask):] if len(writing_mask) else [])
        if pen_mask is not None:
            writing_mask &= pen_mask
        writing_mask &= ~self.joystick_button_mask(pose_x, pose_y)
        return self.pose_to_digit_image(pose_x, pose_y, pose_z, sample_mask=writing_mask)

    def save_session_image(self, output_path):
        """Save the recorded pose trail as a classifier-ready grayscale PNG."""
        if not self.session_data:
            return

        image = self.rows_to_digit_image(self.session_data, trail_length=0)
        plt.imsave(output_path, image, cmap='gray', vmin=0.0, vmax=1.0)
        print(f"Projected character image saved to: {output_path}")
        return output_path, image

    def save_session_metadata(
        self,
        session_name,
        paths,
        start_time,
        end_time,
        data_points,
        prediction=None,
    ):
        """Save a sidecar JSON file with session settings and git metadata."""
        metadata = {
            'session_name': session_name,
            'label': paths['label'],
            'run_id': self.run_id,
            'repetition': paths['repetition'],
            'basename': paths['basename'],
            'created_at': datetime.now().isoformat(),
            'started_at': start_time,
            'ended_at': end_time,
            'sample_count': data_points,
            'paths': {
                'csv': self.path_relative_to_output(paths['csv_path']),
                'png': self.path_relative_to_output(paths['image_path']),
                'json': self.path_relative_to_output(paths['metadata_path']),
            },
            'input_source': self.input_source,
            'command_line': self.command_line,
            'settings': self.manifest_settings(),
            'raw_log': {
                'enabled': bool(self.raw_csv_enabled and self.raw_csv_path),
                'path': self.path_relative_to_output(self.raw_csv_path),
            },
            'prediction': prediction,
            'git': self.collect_git_metadata(),
        }
        try:
            with open(paths['metadata_path'], 'w', encoding='utf-8') as f:
                json.dump(metadata, f, indent=2)
                f.write('\n')
            print(f"Session metadata saved to: {paths['metadata_path']}")
            return paths['metadata_path']
        except IOError as e:
            print(f"Error saving session metadata: {e}")
            return None

    def print_final_prediction(self, character_image):
        """Run one final unsmoothed prediction for a completed recording."""
        if self.classifier is None:
            return None

        try:
            with self.classifier_inference_lock:
                result = self.classifier.predict(character_image.astype(np.float32))
        except Exception as e:
            print(f"Final character prediction failed: {e}")
            return {
                'status': 'failed',
                'error': str(e),
            }

        probabilities = result['probabilities']
        labels = result.get('labels', DEFAULT_CLASSIFIER_LABELS)
        top_two = top_label_indices(probabilities, 2)
        best_index = top_two[0]
        if probabilities[best_index] <= 0.0:
            print("Final character prediction: -- (0.0%)")
            return {
                'status': 'empty',
                'best_label': None,
                'best_confidence': 0.0,
                'runner_up_label': None,
                'runner_up_confidence': 0.0,
                'labels': list(labels),
            }

        runner_text = "--"
        runner_label = None
        runner_confidence = 0.0
        if len(top_two) > 1 and probabilities[top_two[1]] > 0.0:
            runner_label = labels[top_two[1]]
            runner_confidence = float(probabilities[top_two[1]])
            runner_text = f"{labels[top_two[1]]} ({probabilities[top_two[1]] * 100:.1f}%)"
        print(
            "Final character prediction: "
            f"{labels[best_index]} ({probabilities[best_index] * 100:.1f}%), "
            f"runner-up {runner_text}"
        )
        return {
            'status': 'ok',
            'best_label': labels[best_index],
            'best_confidence': float(probabilities[best_index]),
            'runner_up_label': runner_label,
            'runner_up_confidence': runner_confidence,
            'labels': list(labels),
        }

    def get_data_copy(self, tail=None):
        """Get a thread-safe copy of the data buffer, optionally limited to the last N rows.

        Parameters
        ----------
        tail : int or None
            If set, only copy the last *tail* rows (avoids copying 30K rows every frame).
            Pass ``self.display_window`` from the update loop for a 20s rolling window.
        """
        with self.data_lock:
            buf = self.data_buffer
            if tail is not None and len(buf) > tail:
                from itertools import islice
                start = len(buf) - tail
                return list(islice(buf, start, len(buf)))
            return list(buf)

    def setup_touchpad_events(self, fig, ax):
        """Wire Matplotlib pointer and key events into the live app state."""

        def update_position(event, pen_down=None):
            if self.input_source != 'touchpad':
                return
            if event.inaxes is not ax or event.xdata is None or event.ydata is None:
                self.touchpad_state['inside'] = False
                if pen_down is not None:
                    self.touchpad_state['pen_down'] = pen_down
                return

            x = float(np.clip(event.xdata, -self.projection_extent, self.projection_extent))
            y = float(np.clip(event.ydata, -self.projection_extent, self.projection_extent))
            self.touchpad_state['x'] = x
            self.touchpad_state['y'] = y
            self.touchpad_state['inside'] = True
            if pen_down is not None:
                self.touchpad_state['pen_down'] = pen_down

        def on_press(event):
            update_position(event, pen_down=True)

        def on_release(event):
            update_position(event, pen_down=False)

        def on_motion(event):
            update_position(event)

        def on_axes_leave(event):
            if self.input_source != 'touchpad':
                return
            self.touchpad_state['inside'] = False
            self.touchpad_state['pen_down'] = False

        def on_key_press(event):
            raw_key = event.key or ''
            key = raw_key.lower()
            # Shift+L/D/R = mode/reset controls (keep lowercase free for recording)
            if self.input_source == 'touchpad' and raw_key in ('L', 'shift+l'):
                self.activate_classifier_mode('letters')
                self._set_key_feedback("Mode: letters")
            elif self.input_source == 'touchpad' and raw_key in ('D', 'shift+d'):
                self.activate_classifier_mode('digits')
                self._set_key_feedback("Mode: digits")
            elif self.input_source == 'touchpad' and raw_key in ('R', 'shift+r'):
                self.reset_writing_canvas()
                self._set_key_feedback("Canvas reset")
            elif self.input_source == 'touchpad' and key in (' ', 'space'):
                self.touchpad_state['pen_down'] = True
            elif self.input_source == 'touchpad' and raw_key in ('P', 'shift+p'):
                # Shift+P = pen toggle; lowercase p falls through to record letter_P
                self.touchpad_pen_enabled = not self.touchpad_pen_enabled
                if not self.touchpad_pen_enabled:
                    self.touchpad_state['pen_down'] = False
                state = "ON" if self.touchpad_pen_enabled else "OFF"
                self._set_key_feedback(f"Pen: {state}")
            elif self.input_source == 'touchpad' and raw_key in ('S', 'shift+s'):
                # Shift+S = stop & save; lowercase s records letter_S
                self.stop_session()
            elif self.input_source == 'touchpad' and raw_key in ('Q', 'shift+q'):
                # Shift+Q = quit; lowercase q records letter_Q
                print("Quitting...")
                self.stop()
            elif self.input_source == 'touchpad' and raw_key == 's':
                self._set_key_feedback("REC  letter_S")
                self.start_session('letter_S')
            elif self.input_source == 'touchpad' and raw_key == 'q':
                self._set_key_feedback("REC  letter_Q")
                self.start_session('letter_Q')
            else:
                self.handle_keypress(raw_key)

        def on_key_release(event):
            key = (event.key or '').lower()
            if self.input_source == 'touchpad' and key in (' ', 'space'):
                self.touchpad_state['pen_down'] = False

        fig.canvas.mpl_connect('button_press_event', on_press)
        fig.canvas.mpl_connect('button_release_event', on_release)
        fig.canvas.mpl_connect('motion_notify_event', on_motion)
        fig.canvas.mpl_connect('axes_leave_event', on_axes_leave)
        fig.canvas.mpl_connect('key_press_event', on_key_press)
        fig.canvas.mpl_connect('key_release_event', on_key_release)
        # Disconnect matplotlib's built-in key handler (l=log scale, r=reset, etc.)
        # so our key bindings take exclusive control.
        try:
            fig.canvas.mpl_disconnect(fig.canvas.manager.key_press_handler_id)
        except AttributeError:
            pass

    def append_touchpad_sample(self, force=False):
        """Append synthetic rows from the current touchpad state."""
        if self.input_source != 'touchpad':
            return

        now = time.monotonic()
        last_sample_at = self.touchpad_state['last_sample_at']
        if not force and now - last_sample_at < self.touchpad_sample_interval:
            return

        current_x = self.touchpad_state['x']
        current_y = self.touchpad_state['y']
        current_pen_down = bool(self.touchpad_state['pen_down'])

        if force or last_sample_at <= 0.0:
            sample_specs = [(now, current_x, current_y, current_pen_down)]
        else:
            elapsed = max(0.0, now - last_sample_at)
            last_x = self.touchpad_state['last_sample_x']
            last_y = self.touchpad_state['last_sample_y']
            last_pen_down = bool(self.touchpad_state['last_sample_pen_down'])
            movement_distance = ((current_x - last_x) ** 2 + (current_y - last_y) ** 2) ** 0.5
            pixel_spacing = 2.0 * self.projection_extent / max(1, self.image_size - 1)
            stroke_spacing = max(pixel_spacing * 0.75, 1e-6)
            time_sample_count = max(1, int(round(elapsed / self.touchpad_sample_interval)))
            distance_sample_count = max(1, int(np.ceil(movement_distance / stroke_spacing)))
            sample_count = min(max(time_sample_count, distance_sample_count), 32)
            sample_specs = []
            for index in range(1, sample_count + 1):
                fraction = index / sample_count
                sample_time = last_sample_at + elapsed * fraction
                sample_x = last_x + (current_x - last_x) * fraction
                sample_y = last_y + (current_y - last_y) * fraction
                sample_pen_down = current_pen_down or (last_pen_down and fraction < 1.0)
                sample_specs.append((sample_time, sample_x, sample_y, sample_pen_down))

        rows = []
        speed_rows = []
        previous_time = last_sample_at
        previous_x = self.touchpad_state['last_sample_x']
        previous_y = self.touchpad_state['last_sample_y']
        for sample_time, sample_x, sample_y, sample_pen_down in sample_specs:
            if previous_time > 0.0 and sample_time > previous_time:
                distance = ((sample_x - previous_x) ** 2 + (sample_y - previous_y) ** 2) ** 0.5
                sample_speed = distance / (sample_time - previous_time)
            else:
                sample_speed = 0.0
            self.touchpad_last_speed = sample_speed
            self.touchpad_speed_samples.append(sample_speed)

            mag_values = (
                self.synthetic_magnetic_values(sample_x, sample_y, self.touchpad_z)
                if self.touchpad_synthetic_magnetics
                else [0.0] * 48
            )
            wall_timestamp = datetime.fromtimestamp(
                time.time() - max(0.0, now - sample_time)
            ).isoformat()
            rows.append([
                wall_timestamp,
                *mag_values,
                sample_x,
                sample_y,
                self.touchpad_z,
                1.0 if sample_pen_down else 0.0,
                0.0,
                0.0,
            ])
            speed_rows.append([
                wall_timestamp,
                sample_x,
                sample_y,
                self.touchpad_z,
                sample_speed,
                1.0 if sample_pen_down else 0.0,
            ])
            previous_time = sample_time
            previous_x = sample_x
            previous_y = sample_y

        with self.data_lock:
            for row_data in rows:
                self.data_buffer.append(row_data)
                if self.current_session:
                    self.session_data.append(row_data)

        if self.csv_writer:
            for row_data in rows:
                self.csv_writer.writerow(row_data)
        if self.touchpad_speed_log_writer:
            for speed_row in speed_rows:
                self.touchpad_speed_log_writer.writerow(speed_row)
            self.touchpad_speed_log_file.flush()

        self.packet_count += len(rows)
        self.touchpad_state['last_sample_at'] = now
        self.touchpad_state['last_sample_x'] = current_x
        self.touchpad_state['last_sample_y'] = current_y
        self.touchpad_state['last_sample_pen_down'] = current_pen_down

    def synthetic_magnetic_values(self, pose_x, pose_y, pose_z):
        """Generate a simple 4x4 dipole-like magnetic field for touchpad rows."""
        values = synthetic_dipole_values(
            pose_x,
            pose_y,
            pose_z,
            projection_extent=self.projection_extent,
            magnetic_scale=1.0 if self.touchpad_magnetic_calibration else self.touchpad_magnetic_scale,
        )
        if self.touchpad_magnetic_calibration:
            return apply_magnetic_calibration(values, self.touchpad_magnetic_calibration)
        return values

    def touchpad_pen_mask(self, rows):
        """Return the pen-down gate encoded in Pose_mx for synthetic rows."""
        if self.input_source != 'touchpad':
            return None
        if not self.touchpad_pen_enabled:
            return np.zeros(len(rows), dtype=bool)
        if self.touchpad_ink_mode == 'pen':
            return np.array([float(row[-3]) > 0.5 for row in rows], dtype=bool)
        return None

    def joystick_button_mask(self, pose_x, pose_y):
        """Return samples that are inside virtual buttons and should not become ink."""
        if len(pose_x) == 0:
            return np.array([], dtype=bool)
        return np.array(
            [
                self.joystick.button_at(x, y) is not None
                if np.isfinite(x) and np.isfinite(y)
                else False
                for x, y in zip(pose_x, pose_y)
            ],
            dtype=bool,
        )

    def live_pose_rows(self, rows):
        """Return the row window used by live pose, joystick, and OCR views."""
        if self.trail_length and len(rows) > self.trail_length:
            return rows[-self.trail_length:]
        return rows

    def classifier_canvas_active(self):
        """Return whether the live OCR canvas should use reset-driven history."""
        return (
            self.app_controller.mode.value in ('letters', 'digits')
            and self.classification_started_at is not None
        )

    def display_pose_rows(self, rows):
        """Return rows for the live display/OCR canvas."""
        if not self.classifier_canvas_active():
            return self.live_pose_rows(rows)

        selected = []
        for row in rows:
            try:
                timestamp = datetime.fromisoformat(str(row[0]))
            except (TypeError, ValueError):
                selected.append(row)
            else:
                if timestamp >= self.classification_started_at:
                    selected.append(row)
        return selected

    def setup_joystick_artists(self, ax):
        """Draw the virtual joystick layer on the projection axis."""
        ax.set_xlim(-self.projection_extent, self.projection_extent)
        ax.set_ylim(-self.projection_extent, self.projection_extent)
        if self.input_source == 'touchpad':
            ax.set_aspect('auto')
            try:
                ax.set_box_aspect(0.62)
            except Exception:
                pass
        else:
            ax.set_aspect('equal', adjustable='box')
        ax.set_xlabel('')
        ax.set_ylabel('')

        button_artists = {}
        for button in self.joystick.buttons:
            circle = patches.Circle(
                (button.x, button.y),
                button.radius,
                facecolor='#f5f5f5',
                edgecolor='#222222',
                linewidth=1.8,
                alpha=0.82,
                zorder=4,
            )
            ax.add_patch(circle)
            label = ax.text(
                button.x,
                button.y,
                button.name,
                ha='center',
                va='center',
                fontsize=10 if button.action.startswith('choice:') else 11,
                weight='bold',
                zorder=5,
            )
            button_artists[button.name] = {
                'circle': circle,
                'label': label,
            }

        cursor_artist = ax.scatter(
            [],
            [],
            c='#d62728',
            edgecolors='#111111',
            linewidths=0.8,
            s=90,
            zorder=6,
        )
        interface_text = ax.text(
            0.0,
            -0.16,
            self.latest_interface_text,
            transform=ax.transAxes,
            va='top',
            ha='left',
            fontsize=8,
            # Fix 9: white bbox clears old text during blit
            bbox={'facecolor': 'white', 'edgecolor': 'none', 'pad': 0},
        )
        completion_text = ax.text(
            0.5,
            0.96,
            "",
            transform=ax.transAxes,
            va='top',
            ha='center',
            fontsize=13,
            weight='bold',
            color='#1b7f3a',
            bbox={
                'boxstyle': 'round,pad=0.35',
                'facecolor': '#e8f7ee',
                'edgecolor': '#1b7f3a',
                'alpha': 0.94,
            },
            zorder=8,
        )
        return button_artists, cursor_artist, interface_text, completion_text

    def update_joystick_artists(self, button_artists):
        active_button = self.app_controller.active_button
        last_button = self.app_controller.last_event.button.name if self.app_controller.last_event else None

        # Fix 7: Cache last-rendered label text to avoid expensive set_text() calls
        if not hasattr(self, '_cached_button_texts'):
            self._cached_button_texts = {}

        for button in self.joystick.buttons:
            artist_group = button_artists[button.name]
            artist = artist_group['circle']
            label_artist = artist_group['label']
            facecolor = '#f5f5f5'
            label_text = button.name
            if button.action == 'mode:letters' and self.app_controller.mode.value == 'letters':
                facecolor = '#d8ecff'
            elif button.action == 'mode:digits' and self.app_controller.mode.value == 'digits':
                facecolor = '#d8ecff'
            elif button.action.startswith('choice:'):
                try:
                    candidate_index = int(button.action.split(':', 1)[1])
                except (IndexError, ValueError):
                    candidate_index = -1
                if 0 <= candidate_index < len(self.classifier_candidates):
                    candidate_label, candidate_confidence = self.classifier_candidates[candidate_index]
                    label_text = f"{button.name}\n{candidate_label}"
                    facecolor = '#fff3bf' if candidate_confidence > 0.0 else '#f5f5f5'
            elif button.action.startswith('canvas:'):
                facecolor = '#fde2e2' if button.name == last_button else '#ffe8e8'
            elif button.action.startswith('robot:') and button.name == last_button:
                facecolor = '#d8f3dc'

            if button.name == active_button:
                facecolor = '#ffd166'

            if (
                self.action_feedback_button == button.name
                and time.monotonic() < self.action_feedback_until
            ):
                facecolor = '#40b872'

            artist.set_facecolor(facecolor)
            # Only call set_text() when text actually changes (avoids text re-layout each frame)
            prev_text = self._cached_button_texts.get(button.name)
            if label_text != prev_text:
                label_artist.set_text(label_text)
                self._cached_button_texts[button.name] = label_text

    def plot_data(self):
        """Create real-time plot of Bx, By, Bz for the selected sensor."""
        fig = plt.figure(figsize=(18, 10))
        fig.suptitle(
            f'Real-time Magnetometer Data - Sensor {self.plot_sensor} | '
            f'Projection: {self.image_size}x{self.image_size}, last {self.trail_length} samples',
            fontsize=14
        )

        grid = fig.add_gridspec(
            2,
            4,
            width_ratios=[0.9, 1.8, 1.8, 1.0],
            height_ratios=[0.85, 1.7],
        )
        ax1 = fig.add_subplot(grid[0, 0])
        ax2 = fig.add_subplot(grid[0, 1])
        ax3 = fig.add_subplot(grid[0, 2])
        ax4 = fig.add_subplot(grid[1, 0], projection='3d')
        ax5 = fig.add_subplot(grid[1, 1:3])
        ax6 = fig.add_subplot(grid[:, 3])

        # Bx plot
        ax1.set_title(f'Sensor {self.plot_sensor} - Bx')
        ax1.set_xlabel('Time (samples)')
        ax1.set_ylabel('Magnetic Field')
        ax1.grid(True)
        line_bx, = ax1.plot([], [], 'b-', label='Bx', linewidth=2)
        ax1.legend(loc='upper right')

        # By plot
        ax2.set_title(f'Sensor {self.plot_sensor} - By')
        ax2.set_xlabel('Time (samples)')
        ax2.set_ylabel('Magnetic Field')
        ax2.grid(True)
        line_by, = ax2.plot([], [], 'g-', label='By', linewidth=2)
        ax2.legend(loc='upper right')

        # Bz plot
        ax3.set_title(f'Sensor {self.plot_sensor} - Bz')
        ax3.set_xlabel('Time (samples)')
        ax3.set_ylabel('Magnetic Field')
        ax3.grid(True)
        line_bz, = ax3.plot([], [], 'r-', label='Bz', linewidth=2)
        ax3.legend(loc='upper right')

        # 3D magnet trajectory plot from pose coordinates
        ax4.set_title('Magnet Trajectory from Pose X/Y/Z')
        ax4.set_xlabel('X')
        ax4.set_ylabel('Y')
        ax4.set_zlabel('Z')
        ax4.grid(True)
        ax4.view_init(elev=30, azim=135)
        try:
            ax4.set_box_aspect([1, 1, 1])
        except Exception:
            pass
        ax4.set_xlim(-self.projection_extent, self.projection_extent)
        ax4.set_ylim(-self.projection_extent, self.projection_extent)
        ax4.set_zlim(-POSE_LIMIT, POSE_LIMIT)

        # Pre-create persistent 3D artists (avoids expensive clear+redraw every frame)
        hover_scatter = ax4.scatter([], [], [], c='#bbbbbb', s=60, alpha=0.35, label='Hover/lift')
        writing_scatter = ax4.scatter([], [], [], c=[], cmap='plasma', s=150, edgecolors='k', linewidths=0.6, label='Writing')
        trajectory_line, = ax4.plot([], [], [], color='#333333', alpha=0.45, linewidth=1.2)
        current_pos_scatter = ax4.scatter([], [], [], c='red', s=120, label='Current position')
        ax4.legend(loc='best', fontsize='small')

        # Live 2D projection preview for character-classifier style inference.
        projection_image = np.ones((self.image_size, self.image_size), dtype=float)
        image_artist = ax5.imshow(
            projection_image,
            cmap='gray',
            vmin=0.0,
            vmax=1.0,
            interpolation='nearest',
            extent=(-self.projection_extent, self.projection_extent, -self.projection_extent, self.projection_extent),
            origin='upper',
            alpha=0.58,
            zorder=1,
        )
        ax5.set_title('Virtual Joystick + Writing Surface')
        ax5.set_xticks([])
        ax5.set_yticks([])
        if self.input_source == 'touchpad':
            _hint_line1 = "Shift+S = save   Shift+Q = quit   |   A-Z / 0-9 / - / = = record   p/s/q = letter P/S/Q"
            _hint_line2 = "Space / click = draw   Shift+P = pen   |   Shift+L = letters OCR   Shift+D = digits OCR   Shift+R = reset"
            _hint_lines = [_hint_line1, _hint_line2]
        else:
            _hint_lines = ["s = stop & save   q = quit   |   A-Z = letter   0-9 = digit   - = blank   = = still"]
        hint_text = ax5.text(
            0.5, 0.03,
            "\n".join(_hint_lines),
            transform=ax5.transAxes,
            ha='center', va='bottom',
            fontsize=6.5,
            family='monospace',
            color='#222222',
            bbox={'facecolor': 'white', 'edgecolor': '#aaaaaa', 'boxstyle': 'round,pad=0.3', 'alpha': 0.93},
            zorder=9,
        )
        button_artists, cursor_artist, interface_text, completion_text = self.setup_joystick_artists(ax5)
        # Collect button artists for blit. Circles must be redrawn for hover/feedback,
        # and labels must be redrawn too so filled patches do not cover cached text.
        button_circles = [artist_group['circle'] for artist_group in button_artists.values()]
        button_labels = [artist_group['label'] for artist_group in button_artists.values()]
        self.setup_touchpad_events(fig, ax5)

        z_range = (
            f"calibrated {self.z_near:.4f}->{self.z_far:.4f}"
            if self.z_near is not None and self.z_far is not None
            else "relative trail range"
        )
        classifier_labels = display_labels_for(self.classifier_labels)
        display_count = min(12, len(classifier_labels))
        display_indices = list(range(display_count))
        classifier_header_rows = 2.8
        y_positions = np.arange(display_count) + classifier_header_rows
        current_classifier_axis_labels = [classifier_labels[index] for index in display_indices]
        ax6.set_title('EasyOCR Character Classifier')
        ax6.set_xlim(0.0, 1.0)
        ax6.set_ylim(-0.5, display_count + classifier_header_rows - 0.5)
        ax6.set_yticks(y_positions)
        ax6.set_yticklabels(current_classifier_axis_labels, fontsize=8)
        ax6.tick_params(axis='y', which='both', left=False, labelleft=True, pad=2)
        ax6.invert_yaxis()
        ax6.set_xlabel('Confidence')
        prob_bars = ax6.barh(y_positions, np.zeros(display_count), color='#7c9cc8')
        prob_bar_artists = list(prob_bars)
        info_text = ax6.text(
            0.98,
            0.98,
            "Prediction: --\n"
            "Runner-up: --\n"
            "Z: --",
            transform=ax6.transAxes,
            va='top',
            ha='right',
            fontsize=8,
            bbox={
                'boxstyle': 'round,pad=0.25',
                'facecolor': 'white',
                'edgecolor': 'none',
                'alpha': 0.88,
            },
            zorder=8,
        )
        ax6.text(
            0.0,
            -0.28,
            f"X/Y projection, {self.image_size}px, trail {self.trail_length}, Z mode {self.z_close_mode}",
            transform=ax6.transAxes,
            va='top',
            fontsize=8,
            # Fix 9: white bbox clears old text during blit
            bbox={'facecolor': 'white', 'edgecolor': 'none', 'pad': 0},
        )
        if self.classifier is None:
            self.latest_prediction_text = self.classifier_unloaded_prediction_text()
            info_text.set_text(
                f"{self.latest_prediction_text}\n"
                f"{self.latest_runner_up_text}\n"
                "Z: --"
            )

        # tight_layout once at init, not per-frame
        plt.tight_layout()

        sensor_trace_artists = () if self.input_source == 'touchpad' else (
            line_bx,
            line_by,
            line_bz,
        )
        raw_blit_artists = (
            *sensor_trace_artists,
            image_artist,
            hint_text,       # drawn after image so it stays on top of ink
            *button_circles,
            *button_labels,
            cursor_artist,
            interface_text,
            completion_text,
            *prob_bar_artists,
            info_text,
        )
        blit_artists = tuple(
            artist for artist in raw_blit_artists
            if getattr(artist, 'axes', None) is not None
        )
        for artist in blit_artists:
            artist.set_animated(True)

        def update_classifier_axis_labels(labels, indices):
            nonlocal current_classifier_axis_labels

            axis_labels = [labels[index] for index in indices]
            axis_labels.extend([''] * (display_count - len(axis_labels)))
            if axis_labels != current_classifier_axis_labels:
                ax6.set_yticks(y_positions)
                ax6.set_yticklabels(axis_labels, fontsize=8)
                fig.canvas.draw()
                try:
                    ani._blit_cache.clear()
                except (NameError, AttributeError):
                    pass
                current_classifier_axis_labels = axis_labels

        def init_plot():
            line_bx.set_data([], [])
            line_by.set_data([], [])
            line_bz.set_data([], [])
            image_artist.set_data(projection_image)
            cursor_artist.set_offsets(np.empty((0, 2)))
            # 3D artists start empty
            hover_scatter._offsets3d = (np.array([]), np.array([]), np.array([]))
            writing_scatter._offsets3d = (np.array([]), np.array([]), np.array([]))
            trajectory_line.set_data([], [])
            trajectory_line.set_3d_properties([])
            current_pos_scatter._offsets3d = (np.array([]), np.array([]), np.array([]))
            return blit_artists

        # Fix 6: Cache axis limits to avoid 6× matplotlib layout recalculations per frame.
        _cached_xlim = [0.0, 0.0, 0.0]
        _cached_ylim = [(0.0, 0.0), (0.0, 0.0), (0.0, 0.0)]

        def update_plot(frame):
            self.append_touchpad_sample()
            # Fix 4: Only copy the last display_window rows instead of the full 30K buffer.
            data = self.get_data_copy(tail=self.display_window)
            if not data:
                return blit_artists

            if self.input_source != 'touchpad':
                # Extract Bx, By, Bz for selected sensor
                sensor_idx = self.plot_sensor - 1
                bx_data = [row[1 + sensor_idx*3] for row in data]
                by_data = [row[1 + sensor_idx*3 + 1] for row in data]
                bz_data = [row[1 + sensor_idx*3 + 2] for row in data]

                # Update line plots
                x_vals = list(range(len(bx_data)))
                line_bx.set_data(x_vals, bx_data)
                line_by.set_data(x_vals, by_data)
                line_bz.set_data(x_vals, bz_data)

                # Fix 6: Throttle axis limits — only update when data exceeds cached bounds by >10%.
                for ax_index, (ax, data_vals) in enumerate([(ax1, bx_data), (ax2, by_data), (ax3, bz_data)]):
                    if data_vals:
                        n = len(data_vals)
                        y_min = min(data_vals)
                        y_max = max(data_vals)
                        prev_xmax = _cached_xlim[ax_index]
                        prev_ymin, prev_ymax = _cached_ylim[ax_index]
                        # Compute target limits (same formula as before)
                        if y_min == y_max:
                            pad = abs(y_min) * 0.1 or 1.0
                            y_lo, y_hi = y_min - pad, y_max + pad
                        else:
                            y_lo, y_hi = y_min * 1.1, y_max * 1.1
                        # Only call set_xlim/set_ylim if bounds have expanded by >10%
                        x_changed = n > prev_xmax * 1.1 or (prev_xmax == 0.0 and n > 0)
                        y_changed = (
                            y_lo < prev_ymin * 0.9 or y_hi > prev_ymax * 1.1
                            or (prev_ymin == 0.0 and prev_ymax == 0.0 and n > 0)
                        )
                        if x_changed:
                            ax.set_xlim(0, n)
                            _cached_xlim[ax_index] = float(n)
                        if y_changed:
                            ax.set_ylim(y_lo, y_hi)
                            _cached_ylim[ax_index] = (y_lo, y_hi)

            pose_rows = self.display_pose_rows(data)
            pose_x, pose_y, pose_z, timestamps = self.extract_pose_trail(
                pose_rows,
                trail_length=0 if self.classifier_canvas_active() else None,
                include_timestamps=True,
            )
            writing_mask, _, _ = self.writing_sample_mask(
                pose_x,
                pose_y,
                pose_z,
                timestamps,
            )
            pen_mask = self.touchpad_pen_mask(pose_rows)
            if pen_mask is not None:
                writing_mask &= pen_mask
            writing_mask &= ~self.joystick_button_mask(pose_x, pose_y)
            capture_mask = self.classification_sample_mask(pose_rows)
            ocr_mask = writing_mask & capture_mask
            ink_count = int(np.count_nonzero(ocr_mask))
            digit_image = self.live_pose_to_digit_image(
                pose_rows,
                pose_x,
                pose_y,
                pose_z,
                ocr_mask,
            )
            image_artist.set_data(digit_image)
            clear_canvas_requested = False
            if len(pose_x) > 0 and np.isfinite(pose_x[-1]) and np.isfinite(pose_y[-1]):
                cursor_artist.set_offsets([[pose_x[-1], pose_y[-1]]])
                interface_event = self.app_controller.update_cursor(
                    pose_x[-1],
                    pose_y[-1],
                    time.monotonic(),
                )
                if interface_event is not None:
                    clear_canvas_requested = (
                        interface_event.command == 'canvas:reset'
                        or interface_event.classifier_labels is not None
                    )
                    self.handle_interface_event(interface_event)
                    if interface_event.classifier_labels is not None and pose_rows:
                        self.mark_classification_start(pose_rows[-1][0])
            else:
                cursor_artist.set_offsets(np.empty((0, 2)))

            if clear_canvas_requested:
                self.reset_live_digit_image()
                digit_image = self.live_digit_image.copy()
                image_artist.set_data(digit_image)
                ocr_mask = np.zeros_like(ocr_mask, dtype=bool)
                ink_count = 0

            now = time.monotonic()
            self.report_touchpad_speed(now)
            self.update_joystick_artists(button_artists)
            dwell_text = (
                f"{self.app_controller.active_button} "
                f"{self.app_controller.dwell_progress * 100:.0f}%"
                if self.app_controller.active_button
                else "--"
            )
            active_classifier_labels = display_labels_for(self.classifier_labels)
            labels_text = ''.join(active_classifier_labels)
            if len(labels_text) > 12:
                labels_text = labels_text[:12] + "..."
            last_text = self.app_controller.last_command or (
                self.app_controller.last_event.command if self.app_controller.last_event else '--'
            )
            self.latest_interface_text = (
                f"Mode: {self.app_controller.mode.value} | "
                f"Button: {dwell_text} | "
                f"Labels: {labels_text} | "
                f"Last: {last_text}"
            )
            if self.input_source == 'touchpad':
                pen_text = "down" if self.touchpad_state['pen_down'] else "up"
                if self.touchpad_ink_mode == 'pen':
                    self.latest_interface_text += f" | Ink: pen {pen_text}"
                else:
                    self.latest_interface_text += (
                        f" | Ink: velocity "
                        f"{self.touchpad_last_speed:.3f}/{self.writing_min_velocity:.3f}"
                    )
            if self.classifier_candidates:
                choices_text = ' '.join(
                    f"{index + 1}={label}"
                    for index, (label, _) in enumerate(self.classifier_candidates[:4])
                )
                self.latest_interface_text += f" | Choices: {choices_text}"
            if self.current_session:
                self.latest_interface_text += (
                    f" | Rec: {self.current_session} "
                    f"({len(self.session_data)})"
                )
            if now < self.action_feedback_until and self.action_feedback_text:
                self.latest_interface_text += f" | {self.action_feedback_text}"
                completion_text.set_text(self.action_feedback_text)
            else:
                completion_text.set_text("")
            interface_text.set_text(self.latest_interface_text)

            prediction_text = self.latest_prediction_text
            runner_up_text = self.latest_runner_up_text
            is_writing_now = bool(len(ocr_mask) > 0 and ocr_mask[-1])
            can_classify = (
                self.enable_classifier
                and self.app_controller.mode.value in ('letters', 'digits')
                and ink_count >= self.classifier_min_ink_samples
            )
            if can_classify:
                if self.classifier_mode == 'continuous':
                    self.submit_classifier_image(digit_image)
                else:
                    if is_writing_now:
                        if not self.classifier_waiting_for_pause:
                            self.classifier_candidates = []
                        self.classifier_last_writing_time = now
                        self.classifier_waiting_for_pause = True
                    elif (
                        self.classifier_waiting_for_pause
                        and now - self.classifier_last_writing_time >= self.classifier_idle_seconds
                        and ink_count != self.classifier_last_submitted_ink_count
                    ):
                        self.submit_classifier_image(digit_image, force=True)
                        self.classifier_last_submitted_ink_count = ink_count
                        self.classifier_waiting_for_pause = False

            result, classifier_error, classifier_status = self.get_classifier_state()
            if result is None:
                active_display_count = min(display_count, len(active_classifier_labels))
                update_classifier_axis_labels(active_classifier_labels, list(range(active_display_count)))

            if classifier_error is not None:
                self.classifier_candidates = []
                prediction_text = f"Prediction failed: {classifier_error}"
                runner_up_text = "Runner-up: --"
            elif result is not None:
                probabilities = result['probabilities']
                result_labels = tuple(result.get('labels', active_classifier_labels))
                top_two = top_label_indices(probabilities, 2)
                top_four = top_label_indices(probabilities, min(4, len(result_labels)))
                if top_four and probabilities[top_four[0]] > 0.0:
                    self.classifier_candidates = [
                        (result_labels[index], float(probabilities[index]))
                        for index in top_four
                    ]
                else:
                    self.classifier_candidates = []
                active_display_count = min(display_count, len(result_labels))
                # Sort all labels by probability descending so top predictions appear first
                sorted_by_prob = sorted(
                    range(len(result_labels)),
                    key=lambda i: -float(probabilities[i]),
                )
                display_indices = sorted_by_prob[:active_display_count]

                self.update_joystick_artists(button_artists)

                if probabilities[top_two[0]] > 0.0:
                    prediction_text = (
                        f"Prediction: {result_labels[top_two[0]]} "
                        f"({probabilities[top_two[0]] * 100:.1f}%)"
                    )
                    if self.ros_bridge:
                        self.ros_bridge.publish_classifier(
                            result_labels[top_two[0]],
                            float(probabilities[top_two[0]]),
                        )
                else:
                    prediction_text = f"Prediction: -- ({classifier_status})"
                if len(top_two) > 1 and probabilities[top_two[1]] > 0.0:
                    runner_up_text = (
                        f"Runner-up:  {result_labels[top_two[1]]} "
                        f"({probabilities[top_two[1]] * 100:.1f}%)"
                    )
                else:
                    runner_up_text = "Runner-up: --"

                update_classifier_axis_labels(result_labels, display_indices)

                for bar in prob_bars:
                    bar.set_width(0.0)
                    bar.set_color('#7c9cc8')
                for bar, index in zip(prob_bars, display_indices):
                    bar.set_width(float(probabilities[index]))
                    bar.set_color('#2f5f9f' if index == top_two[0] and probabilities[index] > 0.0 else '#7c9cc8')

            if self.classifier is not None and self.app_controller.mode.value not in ('letters', 'digits'):
                prediction_text = f"Prediction: paused ({self.app_controller.mode.value} mode)"
                runner_up_text = "Runner-up: --"
            elif self.enable_classifier and self.classifier is None and self.classifier_loading:
                prediction_text = "Prediction: classifier loading"
                runner_up_text = "Runner-up: --"
            elif self.enable_classifier and self.classifier is None and self.classifier_load_error is not None:
                prediction_text = "Prediction: classifier failed to load"
                runner_up_text = "Runner-up: --"
            elif (
                self.enable_classifier
                and self.app_controller.mode.value in ('letters', 'digits')
                and ink_count < self.classifier_min_ink_samples
            ):
                prediction_text = "Prediction: waiting for writing"
                runner_up_text = "Runner-up: --"
            elif (
                self.enable_classifier
                and self.classifier_mode == 'on-idle'
                and self.app_controller.mode.value in ('letters', 'digits')
                and self.classifier_waiting_for_pause
            ):
                remaining = max(0.0, self.classifier_idle_seconds - (now - self.classifier_last_writing_time))
                prediction_text = f"Prediction: waiting for pause ({remaining:.1f}s)"

            self.latest_prediction_text = prediction_text
            self.latest_runner_up_text = runner_up_text

            if len(pose_z) > 0:
                finite_z = pose_z[np.isfinite(pose_z)]
                if len(finite_z) > 0:
                    info_text.set_text(
                        f"{prediction_text}\n"
                        f"{runner_up_text}\n"
                        f"Ink: {ink_count}/{len(pose_z)}\n"
                        f"Z: {finite_z[-1]:.4f} "
                        f"[{np.min(finite_z):.4f}..{np.max(finite_z):.4f}]"
                    )
            else:
                info_text.set_text(
                    f"{prediction_text}\n"
                    f"{runner_up_text}\n"
                    f"Ink: {ink_count}/{len(pose_z)}\n"
                    "Z: --"
                )

            # The 3D subplot is intentionally not part of the blit set. Matplotlib's
            # mplot3d artists are backend-fragile under blitting and were causing
            # the 2D writing surface to vanish. The live interaction path uses the
            # 2D projection, sensor traces, and classifier panel.
            return blit_artists

        target_plot_fps = self.plot_fps
        if target_plot_fps is None:
            target_plot_fps = 60.0 if self.input_source == 'touchpad' else 30.0
        animation_interval_ms = max(1, int(round(1000.0 / target_plot_fps)))

        # Blit only the 2D artists. The 3D axis stays static because mplot3d
        # does not blit reliably on all Matplotlib backends.
        ani = FuncAnimation(
            fig,
            update_plot,
            init_func=init_plot,
            interval=animation_interval_ms,
            blit=True,
            cache_frame_data=False,
        )
        plt.show()

    def signal_handler(self, signum, frame):
        """Handle Ctrl+C for graceful shutdown."""
        print("\nReceived signal, shutting down...")
        self.stop()

    def stop(self):
        """Stop reading and close all resources."""
        self.is_running = False

        if self.read_thread and self.read_thread.is_alive():
            self.read_thread.join(timeout=2)

        self.classifier_stop_event.set()
        self.classifier_input_event.set()
        if self.classifier_thread and self.classifier_thread.is_alive():
            self.classifier_thread.join(timeout=2)

        if self.classifier_load_thread and self.classifier_load_thread.is_alive():
            self.classifier_load_thread.join(timeout=2)

        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()

        if self.csv_file:
            self.csv_file.close()

        if self.touchpad_speed_log_file:
            self.touchpad_speed_log_file.close()
            self.touchpad_speed_log_file = None
            self.touchpad_speed_log_writer = None

        print("Magnetometer reader stopped")

    def start_touchpad_speed_logging(self, filename):
        """Write touchpad pose speed samples for threshold calibration."""
        try:
            os.makedirs(os.path.dirname(filename) if os.path.dirname(filename) else '.', exist_ok=True)
            self.touchpad_speed_log_file = open(filename, 'w', newline='')
            self.touchpad_speed_log_writer = csv.writer(self.touchpad_speed_log_file)
            self.touchpad_speed_log_writer.writerow([
                'timestamp',
                'pose_x',
                'pose_y',
                'pose_z',
                'xy_speed',
                'pen_down',
            ])
            print(f"Started touchpad speed logging to {filename}")
            return True
        except IOError as e:
            print(f"Error creating touchpad speed log: {e}")
            return False

    def touchpad_speed_summary(self):
        """Return compact speed stats for calibration."""
        speeds = np.asarray(self.touchpad_speed_samples, dtype=float)
        speeds = speeds[np.isfinite(speeds)]
        if len(speeds) == 0:
            return None
        return {
            'last': float(self.touchpad_last_speed),
            'p50': float(np.percentile(speeds, 50)),
            'p90': float(np.percentile(speeds, 90)),
            'p99': float(np.percentile(speeds, 99)),
            'max': float(np.max(speeds)),
        }

    def report_touchpad_speed(self, now=None):
        """Print touchpad speed stats at a controlled cadence."""
        if self.input_source != 'touchpad' or self.touchpad_speed_report_interval <= 0:
            return
        now = time.monotonic() if now is None else now
        if now - self.touchpad_last_speed_report_at < self.touchpad_speed_report_interval:
            return

        summary = self.touchpad_speed_summary()
        if summary is None:
            return
        self.touchpad_last_speed_report_at = now
        print(
            "Touchpad speed "
            f"last={summary['last']:.4f}, "
            f"p50={summary['p50']:.4f}, "
            f"p90={summary['p90']:.4f}, "
            f"p99={summary['p99']:.4f}, "
            f"max={summary['max']:.4f}, "
            f"threshold={self.writing_min_velocity:.4f}"
        )

    def run(self, csv_filename=None, baudrate=921600):
        """Main run function.

        Mode logic:
          dry_run        — no sensor/touchpad, create data layout, then exit
          validation_mode — connect to sensor, print health stats, no files saved
          normal          — live view, sessions saved only when --record-data is set
        """
        try:
            if self.dry_run:
                print("\n=== DRY RUN MODE ===")
                print("No sensor or touchpad connected. Creating data layout only.")
                self.prepare_recording_layout(raw_csv_path=None, raw_csv_enabled=False)
                print(f"Output directory: {self.output_dir}")
                print(f"Manifest: {self.manifest_path}")
                print("Created subfolders: raw/, samples/")
                return

            if self.validation_mode:
                print("\n=== VALIDATION MODE ===")
                print("Sensor health check only — NO files will be saved.")
                # Open port and read for a short period, check packet health
                port_name = self.choose_port()
                if not port_name:
                    return
                if not self.open_port(port_name, baudrate):
                    return
                if not self.start_reading():
                    return
                self.is_running = True
                print("Validating sensor link for 60 seconds (Ctrl+C to skip)...")
                # Show minimal layout for validation
                self.plot_data()
                return

            # === Normal mode ===
            if self.record_data:
                self.prepare_recording_layout(
                    raw_csv_path=csv_filename,
                    raw_csv_enabled=bool(csv_filename),
                )

            if self.input_source == 'serial':
                port_name = self.choose_port()
                if not port_name:
                    return
                if not self.open_port(port_name, baudrate):
                    return
                if csv_filename and not self.start_csv_logging(csv_filename):
                    return
                if not self.start_reading():
                    return
            elif self.input_source == 'touchpad':
                if csv_filename and not self.start_csv_logging(csv_filename):
                    return
                if self.touchpad_speed_log and not self.start_touchpad_speed_logging(self.touchpad_speed_log):
                    return
                self.is_running = True
                self.append_touchpad_sample(force=True)
                print(
                    "Started touchpad simulator "
                    f"({self.touchpad_sample_rate:.1f} Hz, z={self.touchpad_z}, "
                    f"ink={self.touchpad_ink_strength})"
                )
            else:
                print(f"Unknown input source: {self.input_source}")
                return

            print("\n" + "="*60)
            print("MAGNETOMETER DATA ACQUISITION SYSTEM")
            print("="*60)
            print(f"Input source: {self.input_source}")
            if self.record_data:
                self.raw_csv_path = csv_filename
                self.raw_csv_enabled = bool(csv_filename)
                self.write_manifest()
                print(f"RECORDING MODE: Session files saved to {self.output_dir}/samples/")
                print(f"Raw CSV log: {csv_filename if csv_filename else 'disabled (--no-csv)'}")
            else:
                print("Live view only. Use --record-data to enable CSV/PNG session saving.")
            print("Controls:")
            if self.input_source == 'touchpad':
                if self.touchpad_ink_mode == 'pen':
                    print("  Mouse/touchpad : hover to move, click-drag or Space to draw")
                else:
                    print("  Mouse/touchpad : hover to move cursor, movement draws ink")
                print(f"  Touchpad dwell : {self.app_controller.detector.dwell_seconds:.1f}s to activate a button")
                print("  Shift+L        : switch to letters OCR mode")
                print("  Shift+D        : switch to digits OCR mode")
                print("  Shift+R        : reset writing canvas")
                print("  Shift+P        : toggle pen on/off")
                print("  Shift+S        : stop & save current recording")
                print("  Shift+Q        : quit")
                print("  p / s / q      : record letter_P / letter_S / letter_Q")
            else:
                print("  s              : stop & save current recording")
                print("  q              : quit")
            print("  0-9 / A-Z      : start recording that digit / letter")
            print("                   (tip: draw → move to classification button → Shift+S to save)")
            print("  - / =          : start control_blank / control_still recording")
            print(f"Projection: X/Y plane, Z intensity, {self.image_size}x{self.image_size}px, last {self.trail_length} samples")
            if self.enable_writing_filter:
                velocity_range = f"velocity>={self.writing_min_velocity}"
                if self.writing_max_velocity is not None:
                    velocity_range += f" and <= {self.writing_max_velocity}"
                z_gate = (
                    f", closeness>={self.writing_min_closeness}"
                    if self.writing_min_closeness is not None
                    else ", Z gate off"
                )
                print(
                    "Writing filter: "
                    f"{velocity_range}{z_gate}"
                )
            else:
                print("Writing filter: disabled")
            if self.record_data:
                self.print_recording_checklist()
            else:
                print("="*60)
            print("Press Ctrl+C to stop...")

            # Start keyboard input thread
            self.start_keyboard_thread()

            # Start plotting in main thread
            self.plot_data()

        except KeyboardInterrupt:
            pass
        finally:
            self.stop()


def main():
    """Main function to run the magnetometer reader."""
    parser = argparse.ArgumentParser(description='Magnetometer Data Reader for Collaborative Robotics Seminar')
    parser.add_argument('--sensor', '-s', type=int, default=1, choices=range(1, 17),
                       help='Sensor number to plot (1-16, default: 1)')
    parser.add_argument('--baudrate', '-b', type=int, default=921600,
                       help='Serial baudrate (default: 921600)')
    parser.add_argument('--csv', '-c', type=str, default='magnetometer_data.csv',
                       help='Explicit continuous raw CSV output filename. '
                            'Default is off in live mode, or output_dir/raw/<run_id>_raw.csv with --record-data')
    parser.add_argument('--no-csv', action='store_true',
                       help='Disable CSV logging for maximum live read rate')
    parser.add_argument('--input-source', choices=['serial', 'touchpad'], default='serial',
                       help='Input source: real serial magnetometer or synthetic touchpad rows (default: serial)')
    parser.add_argument('--touchpad-z', type=float, default=0.02,
                       help='Synthetic Pose_z value used in --input-source touchpad mode (default: 0.02)')
    parser.add_argument('--touchpad-sample-rate', type=float, default=100.0,
                       help='Synthetic sample rate for --input-source touchpad mode in Hz (default: 100)')
    parser.add_argument('--touchpad-ink-strength', type=float, default=1.0,
                       help='Synthetic stroke darkness for --input-source touchpad mode, 0..1 (default: 1.0)')
    parser.add_argument('--touchpad-ink-mode', choices=['pen', 'velocity'], default='velocity',
                       help='Touchpad ink gate: velocity mimics magnet writing, pen uses click/space/p (default: velocity)')
    parser.add_argument('--no-touchpad-magnetics', action='store_true',
                       help='Use zero magnetic values in touchpad mode instead of synthetic 16-sensor fields')
    parser.add_argument('--touchpad-magnetic-scale', type=float, default=1e-6,
                       help='Scale factor for synthetic magnetic fields in touchpad mode (default: 1e-6)')
    parser.add_argument('--touchpad-magnetic-calibration', type=str, default=None,
                       help='Calibration JSON from calibrate_touchpad_magnetics.py for touchpad synthetic magnetics')
    parser.add_argument('--touchpad-speed-log', type=str, default=None,
                       help='Optional CSV path for touchpad speed calibration samples')
    parser.add_argument('--touchpad-speed-report-interval', type=float, default=1.0,
                       help='Seconds between touchpad speed calibration printouts; use 0 to disable (default: 1.0)')
    parser.add_argument('--display-window', type=int, default=2000,
                       help='Recent samples to show in live plots (default: 2000 serial, 240 touchpad). '
                            'Lower = less data to render per frame = less lag.')
    parser.add_argument('--plot-fps', type=float, default=None,
                       help='Target UI redraw rate. Defaults to 60 FPS for touchpad, 30 FPS for serial')
    parser.add_argument('--verbose', '-v', action='store_true',
                       help='Print every decoded packet for debugging (slows real-time reading)')
    parser.add_argument('--trail-length', type=int, default=250,
                       help='Number of recent pose samples used for the 3D trail and 2D image (default: 250)')
    parser.add_argument('--image-size', type=int, default=64,
                       help='Width and height of saved/live projection images in pixels (default: 64)')
    parser.add_argument('--image-dir', type=str, default='digit_images',
                       help='Directory for projected character PNGs (default: digit_images)')
    parser.add_argument('--projection-extent', type=float, default=POSE_LIMIT,
                       help='Half-width of the X/Y projection area in pose units (default: 0.05)')
    parser.add_argument('--z-close-mode', choices=['min', 'max', 'abs-min'], default='max',
                       help='How Z maps to closeness/darkness: max means larger Z is darker (default)')
    parser.add_argument('--z-near', type=float, default=None,
                       help='Z value for darkest stroke. Use with --z-far for fixed height calibration')
    parser.add_argument('--z-far', type=float, default=None,
                       help='Z value for lightest stroke. Use with --z-near for fixed height calibration')
    parser.add_argument('--no-classifier', action='store_true',
                       help='Skip loading the real-time EasyOCR character classifier')
    parser.add_argument('--classifier-gpu', action='store_true',
                       help='Use GPU for EasyOCR if available')
    parser.add_argument('--classifier-labels', default='alphanumeric',
                       help='EasyOCR label set: digits, letters, alphanumeric, or a custom subset such as ABCXLRUD0123 (default: alphanumeric)')
    parser.add_argument('--load-classifier-at-start', action='store_true',
                       help='Load EasyOCR during startup instead of lazily on the first completed drawing')
    parser.add_argument('--classifier-cpu-threads', type=int, default=1,
                       help='CPU threads used by torch/OpenCV inside EasyOCR; 1 keeps the UI responsive (default: 1)')
    parser.add_argument('--classifier-interval', type=float, default=0.75,
                       help='Minimum seconds between background OCR requests (default: 0.75)')
    parser.add_argument('--classifier-mode', choices=['on-idle', 'continuous'], default='on-idle',
                       help='When to run live OCR: after a writing pause or continuously at --classifier-interval (default: on-idle)')
    parser.add_argument('--classifier-idle-seconds', type=float, default=0.8,
                       help='Writing pause duration before OCR runs in on-idle mode (default: 0.8)')
    parser.add_argument('--classifier-min-ink-samples', type=int, default=8,
                       help='Minimum ink samples required before live OCR can run (default: 8)')
    parser.add_argument('--letter-labels', default='letters',
                       help='Label set used after holding virtual L for letter mode (default: letters)')
    parser.add_argument('--digit-labels', default='digits',
                       help='Label set used after holding virtual R for number mode (default: digits)')
    parser.add_argument('--joystick-dwell-seconds', type=float, default=1.5,
                       help='Seconds the cursor must dwell inside a virtual button to press it (default: 1.5)')
    parser.add_argument('--no-writing-filter', action='store_true',
                       help='Draw every pose sample into the OCR image instead of filtering writing/hover samples')
    parser.add_argument('--writing-min-velocity', type=float, default=0.01,
                       help='Minimum XY pose velocity for a sample to count as writing (default: 0.01 pose-units/s)')
    parser.add_argument('--writing-max-velocity', type=float, default=None,
                       help='Optional maximum XY pose velocity for writing; faster moves become repositioning (default: disabled)')
    parser.add_argument('--writing-min-closeness', type=float, default=None,
                       help='Optional normalized Z closeness gate for board/contact mode, 0..1 (default: disabled for air-writing)')

    # === New mode flags ===
    parser.add_argument('--record-data', action='store_true',
                       help='Explicitly enable saving CSV/PNG/JSON session files (default: off, live view only)')
    parser.add_argument('--ros', action='store_true',
                       help='Publish sensor data, pose and commands to ROS 1 topics (requires rospy / ROS Noetic)')
    parser.add_argument('--dry-run', action='store_true',
                       help='Skip sensor/touchpad setup, create output layout + manifest, and exit')
    parser.add_argument('--validation-mode', action='store_true',
                       help='Connect to sensor and print packet health stats, but do NOT save any files')
    parser.add_argument('--output-dir', type=str, default=None,
                       help='Base output directory for recorded session data; defaults to data/lab_YYYY-MM-DD/')
    parser.add_argument('--run-id', type=str, default=None,
                       help='Optional run label prefix for the shared basename (e.g. run_001)')

    argcomplete.autocomplete(parser)
    args = parser.parse_args()

    if args.trail_length <= 0:
        parser.error("--trail-length must be positive")
    if args.image_size <= 0:
        parser.error("--image-size must be positive")
    if args.projection_extent <= 0:
        parser.error("--projection-extent must be positive")
    if (args.z_near is None) != (args.z_far is None):
        parser.error("--z-near and --z-far must be provided together")
    if args.z_near is not None and args.z_near == args.z_far:
        parser.error("--z-near and --z-far must be different")
    if args.writing_min_velocity < 0:
        parser.error("--writing-min-velocity must be non-negative")
    if args.joystick_dwell_seconds <= 0:
        parser.error("--joystick-dwell-seconds must be positive")
    if args.classifier_interval <= 0:
        parser.error("--classifier-interval must be positive")
    if args.plot_fps is not None and args.plot_fps <= 0:
        parser.error("--plot-fps must be positive")
    if args.classifier_cpu_threads < 1:
        parser.error("--classifier-cpu-threads must be at least 1")
    if args.classifier_idle_seconds <= 0:
        parser.error("--classifier-idle-seconds must be positive")
    if args.classifier_min_ink_samples < 1:
        parser.error("--classifier-min-ink-samples must be at least 1")
    if args.writing_max_velocity is not None and args.writing_max_velocity < args.writing_min_velocity:
        parser.error("--writing-max-velocity must be greater than or equal to --writing-min-velocity")
    if args.writing_min_closeness is not None and not 0.0 <= args.writing_min_closeness <= 1.0:
        parser.error("--writing-min-closeness must be between 0 and 1")
    if args.touchpad_sample_rate <= 0:
        parser.error("--touchpad-sample-rate must be positive")
    if not 0.0 <= args.touchpad_ink_strength <= 1.0:
        parser.error("--touchpad-ink-strength must be between 0 and 1")
    if args.touchpad_magnetic_scale < 0:
        parser.error("--touchpad-magnetic-scale must be non-negative")
    if args.touchpad_magnetic_calibration and args.no_touchpad_magnetics:
        parser.error("--touchpad-magnetic-calibration cannot be combined with --no-touchpad-magnetics")
    if args.touchpad_magnetic_calibration and not os.path.exists(args.touchpad_magnetic_calibration):
        parser.error(f"--touchpad-magnetic-calibration not found: {args.touchpad_magnetic_calibration}")
    if args.touchpad_speed_report_interval < 0:
        parser.error("--touchpad-speed-report-interval must be non-negative")

    # Resolve output_dir with date-based smart default
    if args.output_dir is None:
        args.output_dir = f"data/lab_{datetime.now().strftime('%Y-%m-%d')}"
    if (args.record_data or args.dry_run) and not args.run_id:
        args.run_id = datetime.now().strftime('run_%Y%m%d_%H%M%S')
    if args.run_id:
        args.run_id = MagnetometerReader.sanitize_path_component(args.run_id)

    writing_min_velocity_was_explicit = any(
        arg == '--writing-min-velocity' or arg.startswith('--writing-min-velocity=')
        for arg in sys.argv[1:]
    )
    if (
        args.input_source == 'touchpad'
        and args.touchpad_ink_mode == 'velocity'
        and not writing_min_velocity_was_explicit
    ):
        args.writing_min_velocity = 0.08
    display_window_was_explicit = any(
        arg == '--display-window' or arg.startswith('--display-window=')
        for arg in sys.argv[1:]
    )
    if args.input_source == 'touchpad' and not display_window_was_explicit:
        args.display_window = 240
    classifier_mode_was_explicit = any(
        arg == '--classifier-mode' or arg.startswith('--classifier-mode=')
        for arg in sys.argv[1:]
    )
    classifier_interval_was_explicit = any(
        arg == '--classifier-interval' or arg.startswith('--classifier-interval=')
        for arg in sys.argv[1:]
    )
    if args.input_source == 'touchpad' and not classifier_mode_was_explicit:
        args.classifier_mode = 'continuous'
    if (
        args.input_source == 'touchpad'
        and args.classifier_mode == 'continuous'
        and not classifier_interval_was_explicit
    ):
        args.classifier_interval = 0.35

    print(f"Magnetometer Data Reader")
    if args.dry_run:
        print(f"MODE: DRY RUN — create layout only, output dir: {args.output_dir}")
    elif args.validation_mode:
        print("MODE: VALIDATION — sensor connected, packet health check, NO files saved")
    elif args.record_data:
        print(f"MODE: RECORD — explicit recording, output dir: {args.output_dir}")
    else:
        print("MODE: LIVE VIEW — no data saved, use --record-data to enable recording")
    print(f"Input Source: {args.input_source}")
    print(f"Plotting Sensor: {args.sensor}")
    print(f"Baudrate: {args.baudrate}")
    csv_was_explicit = any(
        arg == '--csv' or arg == '-c' or arg.startswith('--csv=')
        for arg in sys.argv[1:]
    )
    csv_filename = None
    if not args.no_csv and not args.dry_run and not args.validation_mode:
        if csv_was_explicit:
            csv_filename = args.csv
        elif args.record_data:
            csv_filename = os.path.join(args.output_dir, 'raw', f"{args.run_id}_raw.csv")
    print(f"CSV Output: {'disabled' if csv_filename is None else csv_filename}")
    print(f"Verbose Packet Output: {args.verbose}")
    if args.input_source == 'touchpad':
        print(
            f"Touchpad Input: z={args.touchpad_z}, "
            f"sample_rate={args.touchpad_sample_rate} Hz, "
            f"plot_fps={args.plot_fps or 60.0}, "
            f"display_window={args.display_window}, "
            f"ink_strength={args.touchpad_ink_strength}, "
            f"ink_mode={args.touchpad_ink_mode}, "
            f"magnetics={'off' if args.no_touchpad_magnetics else 'synthetic'}, "
            f"calibration={args.touchpad_magnetic_calibration or 'none'}, "
            f"speed_log={args.touchpad_speed_log or 'off'}"
        )
    print(f"Projection Image: {args.image_size}x{args.image_size}px, trail={args.trail_length}, dir={args.image_dir}")
    print(f"Projection Extent: +/-{args.projection_extent}, Z close mode: {args.z_close_mode}")
    if args.z_near is not None:
        print(f"Z Calibration: near={args.z_near}, far={args.z_far}")
    else:
        print("Z Calibration: relative per trail")
    print(
        f"Character Classifier: {'disabled' if args.no_classifier else 'EasyOCR'} "
        f"({args.classifier_labels}, mode={args.classifier_mode}, "
        f"interval={args.classifier_interval}s, lazy={not args.load_classifier_at_start})"
    )
    print(
        "Virtual Joystick: "
        f"L -> letters ({args.letter_labels}), "
        f"R -> numbers ({args.digit_labels}), "
        f"dwell={args.joystick_dwell_seconds}s"
    )
    if args.no_writing_filter:
        print("Writing Filter: disabled")
    else:
        velocity_range = f"velocity>={args.writing_min_velocity}"
        if args.writing_max_velocity is not None:
            velocity_range += f" and <= {args.writing_max_velocity}"
        z_gate = (
            f", closeness>={args.writing_min_closeness}"
            if args.writing_min_closeness is not None
            else ", Z gate off"
        )
        print(
            "Writing Filter: "
            f"{velocity_range}{z_gate}"
        )
    print("-" * 40)

    reader = MagnetometerReader(
        plot_sensor=args.sensor,
        verbose=args.verbose,
        trail_length=args.trail_length,
        image_size=args.image_size,
        projection_extent=args.projection_extent,
        z_close_mode=args.z_close_mode,
        z_near=args.z_near,
        z_far=args.z_far,
        image_output_dir=args.image_dir,
        enable_classifier=not args.no_classifier,
        classifier_gpu=args.classifier_gpu,
        classifier_labels=args.classifier_labels,
        lazy_classifier=not args.load_classifier_at_start,
        classifier_cpu_threads=args.classifier_cpu_threads,
        classifier_interval=args.classifier_interval,
        classifier_mode=args.classifier_mode,
        classifier_idle_seconds=args.classifier_idle_seconds,
        classifier_min_ink_samples=args.classifier_min_ink_samples,
        input_source=args.input_source,
        touchpad_z=args.touchpad_z,
        touchpad_sample_rate=args.touchpad_sample_rate,
        touchpad_ink_strength=args.touchpad_ink_strength,
        display_window=args.display_window,
        plot_fps=args.plot_fps,
        touchpad_synthetic_magnetics=not args.no_touchpad_magnetics,
        touchpad_magnetic_scale=args.touchpad_magnetic_scale,
        touchpad_ink_mode=args.touchpad_ink_mode,
        touchpad_magnetic_calibration=args.touchpad_magnetic_calibration,
        touchpad_speed_log=args.touchpad_speed_log,
        touchpad_speed_report_interval=args.touchpad_speed_report_interval,
        enable_writing_filter=not args.no_writing_filter,
        writing_min_velocity=args.writing_min_velocity,
        writing_max_velocity=args.writing_max_velocity,
        writing_min_closeness=args.writing_min_closeness,
        joystick_dwell_seconds=args.joystick_dwell_seconds,
        letter_labels=args.letter_labels,
        digit_labels=args.digit_labels,
        # New mode params
        record_data=args.record_data,
        dry_run=args.dry_run,
        validation_mode=args.validation_mode,
        output_dir=args.output_dir,
        run_id=args.run_id,
        ros=args.ros,
    )
    reader.run(csv_filename=csv_filename, baudrate=args.baudrate)


if __name__ == "__main__":
    main()
