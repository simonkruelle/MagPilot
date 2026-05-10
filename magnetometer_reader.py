#!/usr/bin/env python3
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

os.environ.setdefault('XDG_CACHE_HOME', '/private/tmp/colmag-cache')
os.environ.setdefault('MPLCONFIGDIR', '/private/tmp/colmag-matplotlib-cache')

import matplotlib.pyplot as plt
from matplotlib import patches
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import signal
import sys
from collections import deque
import argparse
from datetime import datetime
import select

from colmag.interaction import AppController, VirtualJoystick

try:
    import serial
    import serial.tools.list_ports
except ImportError as exc:
    serial = None
    SERIAL_IMPORT_ERROR = exc
else:
    SERIAL_IMPORT_ERROR = None

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
        writing_min_velocity=0.002,
        writing_max_velocity=None,
        writing_min_closeness=None,
        joystick_dwell_seconds=2.0,
        letter_labels='letters',
        digit_labels='digits',
        classifier_interval=0.75,
        classifier_mode='on-idle',
        classifier_idle_seconds=0.8,
        classifier_min_ink_samples=8,
    ):
        self.serial_port = None
        self.is_running = False
        self.read_thread = None
        self.data_lock = threading.Lock()
        self.data_buffer = deque(maxlen=1000)  # Keep last 1000 readings for plotting
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
            self.latest_prediction_text = "Prediction: classifier ready on first use"

        # Session recording
        self.recording_sessions = []  # List of (session_name, start_time, end_time, data_points)
        self.current_session = None
        self.session_data = []

        # Setup signal handler for graceful shutdown
        signal.signal(signal.SIGINT, self.signal_handler)

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

    def mark_classification_start(self, timestamp):
        """Start a fresh writing capture window for the active OCR mode."""
        try:
            self.classification_started_at = datetime.fromisoformat(str(timestamp))
        except (TypeError, ValueError):
            self.classification_started_at = None
        self.classifier_last_submitted_ink_count = -1
        self.classifier_waiting_for_pause = False

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

    def set_classifier_labels(self, labels):
        """Switch the OCR allowlist without reloading EasyOCR weights."""
        self.classifier_labels = labels
        self.classifier_requested_labels = labels
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
                self.latest_prediction_text = "Prediction: classifier failed to load"
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
            print(f"Virtual joystick: {event.button.name} -> {event.command}")

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

            # Write header
            header = ['timestamp']
            for i in range(16):
                header.extend([f'Sensor{i+1}_Bx', f'Sensor{i+1}_By', f'Sensor{i+1}_Bz'])
            header.extend(['Pose_x', 'Pose_y', 'Pose_z', 'Pose_mx', 'Pose_my', 'Pose_mz'])
            self.csv_writer.writerow(header)

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
                status = f"Read rate: {packets_since_last / (now - last_status_time):.1f} packets/s"
                if self.current_session:
                    status += f" | RECORDING: {self.current_session} ({len(self.session_data)} points)"
                if self.bad_packet_count:
                    status += f" | bad packets: {self.bad_packet_count}"
                print(status)
                last_status_time = now
                last_status_count = self.packet_count

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

    def handle_keypress(self, key):
        """Handle keyboard input for session controls."""
        if key in '0123456789':
            self.start_session(f'digit_{key}')
        elif key == 's':
            self.stop_session()
        elif key == 'q':
            print("Quitting...")
            self.stop()
        elif key.isalpha() and len(key) == 1:
            self.start_session(f'letter_{key.upper()}')

    def start_session(self, session_name):
        """Start recording a new session."""
        if self.current_session:
            print(f"Already recording session '{self.current_session}'. Stop it first with 's'.")
            return

        self.current_session = session_name
        self.session_data = []
        self.prediction_history.clear()
        if self.classifier is not None:
            self.latest_prediction_text = "Prediction: --"
        elif self.enable_classifier:
            self.latest_prediction_text = "Prediction: classifier failed to load"
        else:
            self.latest_prediction_text = "Prediction: classifier disabled"
        self.latest_runner_up_text = "Runner-up: --"
        print(f"\n🎬 STARTED RECORDING SESSION: '{session_name}'")
        print(f"Recording to main CSV file: {self.csv_file.name if self.csv_file else 'None'}")
        print(f"Projected character image will be saved in: {self.image_output_dir}")

    def stop_session(self):
        """Stop the current recording session."""
        if not self.current_session:
            print("No active recording session to stop.")
            return

        session_name = self.current_session
        data_points = len(self.session_data)

        # Save session info
        start_time = self.session_data[0][0] if self.session_data else datetime.now().isoformat()
        end_time = self.session_data[-1][0] if self.session_data else datetime.now().isoformat()

        self.recording_sessions.append((session_name, start_time, end_time, data_points))

        print(f"\n⏹️ STOPPED RECORDING SESSION: '{session_name}'")
        print(f"Recorded {data_points} data points")
        print(f"Duration: {start_time} to {end_time}")

        # Save session data to separate CSV file
        self.save_session_csv(session_name)
        image_result = self.save_session_image(session_name)
        if image_result:
            _, character_image = image_result
            self.print_final_prediction(character_image)

        self.current_session = None
        self.session_data = []

    def save_session_csv(self, session_name):
        """Save session data to a separate CSV file."""
        if not self.session_data:
            return

        filename = f"{session_name}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        try:
            with open(filename, 'w', newline='') as f:
                writer = csv.writer(f)

                # Write header
                header = ['timestamp']
                for i in range(16):
                    header.extend([f'Sensor{i+1}_Bx', f'Sensor{i+1}_By', f'Sensor{i+1}_Bz'])
                header.extend(['Pose_x', 'Pose_y', 'Pose_z', 'Pose_mx', 'Pose_my', 'Pose_mz'])
                writer.writerow(header)

                # Write session data
                for row in self.session_data:
                    writer.writerow(row)

            print(f"Session data saved to: {filename}")

        except IOError as e:
            print(f"Error saving session CSV: {e}")

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

    def pose_to_digit_image(self, pose_x, pose_y, pose_z, sample_mask=None):
        """
        Project a 3D pose trail to a grayscale image.

        X/Y select the pixel location. Z controls stroke darkness: points closer
        to the sensor grid are darker, while untouched pixels remain white.
        """
        image = np.ones((self.image_size, self.image_size), dtype=float)
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

        brush_radius = max(1.25, self.image_size / 32.0)
        brush_extent = int(np.ceil(brush_radius * 2.5))

        for x, y, close in zip(x_pixels, y_pixels, closeness):
            stroke_strength = 0.18 + 0.82 * float(np.clip(close, 0.0, 1.0))

            x0 = max(0, x - brush_extent)
            x1 = min(self.image_size, x + brush_extent + 1)
            y0 = max(0, y - brush_extent)
            y1 = min(self.image_size, y + brush_extent + 1)

            yy, xx = np.ogrid[y0:y1, x0:x1]
            dist_sq = (xx - x) ** 2 + (yy - y) ** 2
            brush = np.exp(-dist_sq / (2 * brush_radius ** 2))
            patch = 1.0 - stroke_strength * brush
            image[y0:y1, x0:x1] = np.minimum(image[y0:y1, x0:x1], patch)

        return np.clip(image, 0.0, 1.0)

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
        return self.pose_to_digit_image(pose_x, pose_y, pose_z, sample_mask=writing_mask)

    def save_session_image(self, session_name):
        """Save the recorded pose trail as a classifier-ready grayscale PNG."""
        if not self.session_data:
            return

        image = self.rows_to_digit_image(self.session_data)
        output_dir = self.image_output_dir
        os.makedirs(output_dir, exist_ok=True)
        filename = f"{session_name}_{datetime.now().strftime('%Y%m%d_%H%M%S')}_{self.image_size}px.png"
        output_path = os.path.join(output_dir, filename)
        plt.imsave(output_path, image, cmap='gray', vmin=0.0, vmax=1.0)
        print(f"Projected character image saved to: {output_path}")
        self.save_session_metadata(session_name, output_path)
        return output_path, image

    def save_session_metadata(self, session_name, image_path):
        """Save a sidecar file describing how the session image was generated."""
        metadata = {
            'session_name': session_name,
            'created_at': datetime.now().isoformat(),
            'image_path': image_path,
            'sample_count': len(self.session_data),
            'image_size': self.image_size,
            'trail_length': self.trail_length,
            'projection_extent': self.projection_extent,
            'z_close_mode': self.z_close_mode,
            'z_near': self.z_near,
            'z_far': self.z_far,
            'classifier_labels': ''.join(display_labels_for(self.classifier_labels)),
            'writing_filter': {
                'enabled': self.enable_writing_filter,
                'min_velocity': self.writing_min_velocity,
                'max_velocity': self.writing_max_velocity,
                'min_closeness': self.writing_min_closeness,
            },
        }
        metadata_path = os.path.splitext(image_path)[0] + '.json'
        try:
            with open(metadata_path, 'w', encoding='utf-8') as f:
                json.dump(metadata, f, indent=2)
            print(f"Session metadata saved to: {metadata_path}")
        except IOError as e:
            print(f"Error saving session metadata: {e}")

    def print_final_prediction(self, character_image):
        """Run one final unsmoothed prediction for a completed recording."""
        if self.classifier is None:
            return

        try:
            with self.classifier_inference_lock:
                result = self.classifier.predict(character_image.astype(np.float32))
        except Exception as e:
            print(f"Final character prediction failed: {e}")
            return

        probabilities = result['probabilities']
        labels = result.get('labels', DEFAULT_CLASSIFIER_LABELS)
        top_two = top_label_indices(probabilities, 2)
        best_index = top_two[0]
        if probabilities[best_index] <= 0.0:
            print("Final character prediction: -- (0.0%)")
            return

        runner_text = "--"
        if len(top_two) > 1 and probabilities[top_two[1]] > 0.0:
            runner_text = f"{labels[top_two[1]]} ({probabilities[top_two[1]] * 100:.1f}%)"
        print(
            "Final character prediction: "
            f"{labels[best_index]} ({probabilities[best_index] * 100:.1f}%), "
            f"runner-up {runner_text}"
        )

    def get_data_copy(self):
        """Get a copy of the data buffer for plotting (thread-safe)."""
        with self.data_lock:
            return list(self.data_buffer)

    def setup_joystick_artists(self, ax):
        """Draw the virtual joystick layer on the projection axis."""
        ax.set_xlim(-self.projection_extent, self.projection_extent)
        ax.set_ylim(-self.projection_extent, self.projection_extent)
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
            ax.text(
                button.x,
                button.y,
                button.name,
                ha='center',
                va='center',
                fontsize=11,
                weight='bold',
                zorder=5,
            )
            button_artists[button.name] = circle

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
        )
        return button_artists, cursor_artist, interface_text

    def update_joystick_artists(self, button_artists):
        active_button = self.app_controller.active_button
        last_button = self.app_controller.last_event.button.name if self.app_controller.last_event else None

        for button in self.joystick.buttons:
            artist = button_artists[button.name]
            facecolor = '#f5f5f5'
            if button.action == 'mode:letters' and self.app_controller.mode.value == 'letters':
                facecolor = '#d8ecff'
            elif button.action == 'mode:digits' and self.app_controller.mode.value == 'digits':
                facecolor = '#d8ecff'
            elif button.action.startswith('robot:') and button.name == last_button:
                facecolor = '#d8f3dc'

            if button.name == active_button:
                facecolor = '#ffd166'

            artist.set_facecolor(facecolor)

    def plot_data(self):
        """Create real-time plot of Bx, By, Bz for the selected sensor."""
        fig = plt.figure(figsize=(14, 8))
        fig.suptitle(
            f'Real-time Magnetometer Data - Sensor {self.plot_sensor} | '
            f'Projection: {self.image_size}x{self.image_size}, last {self.trail_length} samples',
            fontsize=14
        )

        ax1 = fig.add_subplot(2, 3, 1)
        ax2 = fig.add_subplot(2, 3, 2)
        ax3 = fig.add_subplot(2, 3, 3)
        ax4 = fig.add_subplot(2, 3, 4, projection='3d')
        ax5 = fig.add_subplot(2, 3, 5)
        ax6 = fig.add_subplot(2, 3, 6)

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
        button_artists, cursor_artist, interface_text = self.setup_joystick_artists(ax5)

        z_range = (
            f"calibrated {self.z_near:.4f}->{self.z_far:.4f}"
            if self.z_near is not None and self.z_far is not None
            else "relative trail range"
        )
        classifier_labels = display_labels_for(self.classifier_labels)
        display_count = min(12, len(classifier_labels))
        display_indices = list(range(display_count))
        y_positions = np.arange(display_count)
        ax6.set_title('EasyOCR Character Classifier')
        ax6.set_xlim(0.0, 1.0)
        ax6.set_ylim(-0.5, display_count - 0.5)
        ax6.set_yticks(y_positions)
        ax6.set_yticklabels([classifier_labels[index] for index in display_indices])
        ax6.invert_yaxis()
        ax6.set_xlabel('Confidence')
        prob_bars = ax6.barh(y_positions, np.zeros(display_count), color='#7c9cc8')
        info_text = ax6.text(
            0.0,
            1.28,
            "Prediction: --\n"
            "Runner-up: --\n"
            f"Z: -- ({z_range})",
            transform=ax6.transAxes,
            va='top',
            fontsize=9,
        )
        ax6.text(
            0.0,
            -0.28,
            f"X/Y projection, {self.image_size}px, trail {self.trail_length}, Z mode {self.z_close_mode}",
            transform=ax6.transAxes,
            va='top',
            fontsize=8,
        )
        if self.classifier is None and self.enable_classifier and self.classifier_loading:
            self.latest_prediction_text = "Prediction: classifier loading"
        elif self.classifier is None:
            if self.enable_classifier:
                if self.classifier_load_error:
                    self.latest_prediction_text = "Prediction: classifier failed to load"
                else:
                    self.latest_prediction_text = "Prediction: classifier ready on first use"
            else:
                self.latest_prediction_text = "Prediction: classifier disabled"
            info_text.set_text(
                f"{self.latest_prediction_text}\n"
                f"{self.latest_runner_up_text}\n"
                f"Z: -- ({z_range})"
            )

        # tight_layout once at init, not per-frame
        plt.tight_layout()

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
            return (line_bx, line_by, line_bz, image_artist, cursor_artist,
                    hover_scatter, writing_scatter, trajectory_line, current_pos_scatter,
                    interface_text, info_text)

        def update_plot(frame):
            data = self.get_data_copy()
            if not data:
                return (line_bx, line_by, line_bz, image_artist, cursor_artist,
                        hover_scatter, writing_scatter, trajectory_line, current_pos_scatter,
                        interface_text, info_text)

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

            # Update axis limits (legends are set once at init)
            for ax, data_vals in [(ax1, bx_data), (ax2, by_data), (ax3, bz_data)]:
                if data_vals:
                    ax.set_xlim(0, len(data_vals))
                    y_min = min(data_vals)
                    y_max = max(data_vals)
                    if y_min == y_max:
                        pad = abs(y_min) * 0.1 or 1.0
                        ax.set_ylim(y_min - pad, y_max + pad)
                    else:
                        ax.set_ylim(y_min * 1.1, y_max * 1.1)

            pose_x, pose_y, pose_z, timestamps = self.extract_pose_trail(data, include_timestamps=True)
            writing_mask, _, _ = self.writing_sample_mask(
                pose_x,
                pose_y,
                pose_z,
                timestamps,
            )
            capture_mask = self.classification_sample_mask(data)
            ocr_mask = writing_mask & capture_mask
            ink_count = int(np.count_nonzero(ocr_mask))
            digit_image = self.pose_to_digit_image(
                pose_x,
                pose_y,
                pose_z,
                sample_mask=ocr_mask,
            )
            image_artist.set_data(digit_image)
            if len(pose_x) > 0 and np.isfinite(pose_x[-1]) and np.isfinite(pose_y[-1]):
                cursor_artist.set_offsets([[pose_x[-1], pose_y[-1]]])
                interface_event = self.app_controller.update_cursor(
                    pose_x[-1],
                    pose_y[-1],
                    time.monotonic(),
                )
                if interface_event is not None:
                    self.handle_interface_event(interface_event)
                    if interface_event.classifier_labels is not None and data:
                        self.mark_classification_start(data[-1][0])
            else:
                cursor_artist.set_offsets(np.empty((0, 2)))

            self.update_joystick_artists(button_artists)
            dwell_text = (
                f"{self.app_controller.active_button} "
                f"{self.app_controller.dwell_progress * 100:.0f}%"
                if self.app_controller.active_button
                else "--"
            )
            labels_text = ''.join(display_labels_for(self.classifier_labels))
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
            interface_text.set_text(self.latest_interface_text)

            prediction_text = self.latest_prediction_text
            runner_up_text = self.latest_runner_up_text
            now = time.monotonic()
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
            if classifier_error is not None:
                prediction_text = f"Prediction failed: {classifier_error}"
                runner_up_text = "Runner-up: --"
            elif result is not None:
                probabilities = result['probabilities']
                result_labels = result.get('labels', classifier_labels)
                top_two = top_label_indices(probabilities, 2)
                active_display_count = min(display_count, len(result_labels))
                display_indices = top_label_indices(probabilities, active_display_count)

                if probabilities[top_two[0]] > 0.0:
                    prediction_text = (
                        f"Prediction: {result_labels[top_two[0]]} "
                        f"({probabilities[top_two[0]] * 100:.1f}%)"
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

                # Only update tick labels when they actually change (Fix 4)
                if result_labels != classifier_labels:
                    axis_labels = [result_labels[index] for index in display_indices]
                    axis_labels.extend([''] * (display_count - len(axis_labels)))
                    ax6.set_yticklabels(axis_labels)

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
                        f"Ink: {ink_count}/{len(pose_z)} | "
                        f"Z: {finite_z[-1]:.5f} [{np.min(finite_z):.5f}, {np.max(finite_z):.5f}]"
                    )
            else:
                info_text.set_text(
                    f"{prediction_text}\n"
                    f"{runner_up_text}\n"
                    f"Ink: {ink_count}/{len(pose_z)} | Z: -- ({z_range})"
                )

            # Update persistent 3D artists in-place instead of clear+redraw (Fix 1)
            if len(pose_x) > 1:
                colors = np.linspace(0, 1, len(pose_x))
                hover_mask = ~writing_mask

                if np.any(hover_mask):
                    hover_scatter._offsets3d = (
                        pose_x[hover_mask], pose_y[hover_mask], pose_z[hover_mask]
                    )
                    hover_scatter.set_visible(True)
                else:
                    hover_scatter.set_visible(False)

                if np.any(writing_mask):
                    writing_scatter._offsets3d = (
                        pose_x[writing_mask], pose_y[writing_mask], pose_z[writing_mask]
                    )
                    writing_scatter.set_array(colors[writing_mask])
                    writing_scatter.set_visible(True)
                else:
                    writing_scatter.set_visible(False)

                trajectory_line.set_data(pose_x, pose_y)
                trajectory_line.set_3d_properties(pose_z)

                current_pos_scatter._offsets3d = (
                    pose_x[-1:], pose_y[-1:], pose_z[-1:]
                )
                current_pos_scatter.set_visible(True)
            else:
                hover_scatter.set_visible(False)
                writing_scatter.set_visible(False)
                trajectory_line.set_data([], [])
                trajectory_line.set_3d_properties([])
                current_pos_scatter.set_visible(False)

            # tight_layout removed from per-frame path (Fix 2)
            # Legend calls removed from per-frame path (Fix 3)
            return (line_bx, line_by, line_bz, image_artist, cursor_artist,
                    hover_scatter, writing_scatter, trajectory_line, current_pos_scatter,
                    interface_text, info_text)

        ani = FuncAnimation(
            fig,
            update_plot,
            init_func=init_plot,
            interval=100,
            blit=True,   # Only redraw changed artists (Fix 5)
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

        print("Magnetometer reader stopped")

    def run(self, csv_filename='magnetometer_data.csv', baudrate=921600):
        """Main run function."""
        try:
            # List and choose port
            port_name = self.choose_port()
            if not port_name:
                return

            # Open serial port
            if not self.open_port(port_name, baudrate):
                return

            # Start CSV logging
            if csv_filename and not self.start_csv_logging(csv_filename):
                return

            # Start reading thread
            if not self.start_reading():
                return

            print("\n" + "="*60)
            print("MAGNETOMETER DATA ACQUISITION SYSTEM")
            print("="*60)
            print("Controls:")
            print("  0-9: Start recording that digit")
            print("  A-Z: Start recording that letter (lowercase s/q are reserved)")
            print("  s:   Stop current recording and save CSV + projected PNG")
            print("  q:   Quit program")
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
                       help='CSV output filename (default: magnetometer_data.csv)')
    parser.add_argument('--no-csv', action='store_true',
                       help='Disable CSV logging for maximum live read rate')
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
                       help='EasyOCR label set: digits, letters, alphanumeric, or a custom subset such as ABCX0123 (default: alphanumeric)')
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
    parser.add_argument('--joystick-dwell-seconds', type=float, default=2.0,
                       help='Seconds the cursor must dwell inside a virtual button to press it (default: 2.0)')
    parser.add_argument('--no-writing-filter', action='store_true',
                       help='Draw every pose sample into the OCR image instead of filtering writing/hover samples')
    parser.add_argument('--writing-min-velocity', type=float, default=0.002,
                       help='Minimum XY pose velocity for a sample to count as writing (default: 0.002 pose-units/s)')
    parser.add_argument('--writing-max-velocity', type=float, default=None,
                       help='Optional maximum XY pose velocity for writing; faster moves become repositioning (default: disabled)')
    parser.add_argument('--writing-min-closeness', type=float, default=None,
                       help='Optional normalized Z closeness gate for board/contact mode, 0..1 (default: disabled for air-writing)')

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

    print(f"Magnetometer Data Reader")
    print(f"Plotting Sensor: {args.sensor}")
    print(f"Baudrate: {args.baudrate}")
    print(f"CSV Output: {'disabled' if args.no_csv else args.csv}")
    print(f"Verbose Packet Output: {args.verbose}")
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
        enable_writing_filter=not args.no_writing_filter,
        writing_min_velocity=args.writing_min_velocity,
        writing_max_velocity=args.writing_max_velocity,
        writing_min_closeness=args.writing_min_closeness,
        joystick_dwell_seconds=args.joystick_dwell_seconds,
        letter_labels=args.letter_labels,
        digit_labels=args.digit_labels,
    )
    reader.run(csv_filename=None if args.no_csv else args.csv, baudrate=args.baudrate)


if __name__ == "__main__":
    main()
