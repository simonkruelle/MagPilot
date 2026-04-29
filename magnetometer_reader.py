#!/usr/bin/env python3
"""
Magnetometer Data Reader for Collaborative Robotics Seminar
Reads serial data from magnetometer sensor and processes it.
"""

import serial
import serial.tools.list_ports
import threading
import time
import struct
import csv
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import signal
import sys
from collections import deque
import argparse
import os
from datetime import datetime
import select

PACKET_SIZE = 218
PACKET_HEADER = 0xAA
PACKET_TAIL = 0xBB
POSE_LIMIT = 0.05


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
        self.classifier_gpu = classifier_gpu
        self._classifier_frame_skip = 0

        if enable_classifier:
            self.load_classifier()

        # Session recording
        self.recording_sessions = []  # List of (session_name, start_time, end_time, data_points)
        self.current_session = None
        self.session_data = []

        # Setup signal handler for graceful shutdown
        signal.signal(signal.SIGINT, self.signal_handler)

    def load_classifier(self):
        """Load the optional pretrained EasyOCR digit classifier."""
        try:
            from digit_classifier.inference import DigitClassifier

            self.classifier = DigitClassifier(gpu=self.classifier_gpu)
            print("Loaded EasyOCR digit classifier")
        except Exception as e:
            self.classifier = None
            print(f"Digit classifier not loaded: {e}")

    def list_ports(self):
        """List all available serial ports."""
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
        except serial.SerialException as e:
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

                except serial.SerialException as e:
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
                    key = sys.stdin.read(1).strip().lower()
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

    def start_session(self, session_name):
        """Start recording a new session."""
        if self.current_session:
            print(f"Already recording session '{self.current_session}'. Stop it first with 's'.")
            return

        self.current_session = session_name
        self.session_data = []
        self.prediction_history.clear()
        print(f"\n🎬 STARTED RECORDING SESSION: '{session_name}'")
        print(f"Recording to main CSV file: {self.csv_file.name if self.csv_file else 'None'}")
        print(f"Projected digit image will be saved in: {self.image_output_dir}")

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
            _, digit_image = image_result
            self.print_final_prediction(digit_image)

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

    def extract_pose_trail(self, rows, trail_length=None):
        """Return pose X/Y/Z arrays from buffered CSV-style rows."""
        if not rows:
            return np.array([]), np.array([]), np.array([])

        limit = trail_length if trail_length is not None else self.trail_length
        rows = rows[-limit:] if limit and len(rows) > limit else rows

        pose_x = np.array([row[-6] for row in rows], dtype=float)
        pose_y = np.array([row[-5] for row in rows], dtype=float)
        pose_z = np.array([row[-4] for row in rows], dtype=float)
        return pose_x, pose_y, pose_z

    def pose_to_digit_image(self, pose_x, pose_y, pose_z):
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
        pose_x, pose_y, pose_z = self.extract_pose_trail(rows, trail_length=trail_length)
        return self.pose_to_digit_image(pose_x, pose_y, pose_z)

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
        print(f"Projected digit image saved to: {output_path}")
        return output_path, image

    def print_final_prediction(self, digit_image):
        """Run one final unsmoothed prediction for a completed recording."""
        if self.classifier is None:
            return

        try:
            result = self.classifier.predict(digit_image.astype(np.float32))
        except Exception as e:
            print(f"Final digit prediction failed: {e}")
            return

        probabilities = result['probabilities']
        top_two = np.argsort(probabilities)[-2:][::-1]
        print(
            "Final digit prediction: "
            f"{int(top_two[0])} ({probabilities[top_two[0]] * 100:.1f}%), "
            f"runner-up {int(top_two[1])} ({probabilities[top_two[1]] * 100:.1f}%)"
        )

    def get_data_copy(self):
        """Get a copy of the data buffer for plotting (thread-safe)."""
        with self.data_lock:
            return list(self.data_buffer)

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

        # By plot
        ax2.set_title(f'Sensor {self.plot_sensor} - By')
        ax2.set_xlabel('Time (samples)')
        ax2.set_ylabel('Magnetic Field')
        ax2.grid(True)
        line_by, = ax2.plot([], [], 'g-', label='By', linewidth=2)

        # Bz plot
        ax3.set_title(f'Sensor {self.plot_sensor} - Bz')
        ax3.set_xlabel('Time (samples)')
        ax3.set_ylabel('Magnetic Field')
        ax3.grid(True)
        line_bz, = ax3.plot([], [], 'r-', label='Bz', linewidth=2)

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

        # Live 2D projection preview for digit-classifier style inference.
        projection_image = np.ones((self.image_size, self.image_size), dtype=float)
        image_artist = ax5.imshow(projection_image, cmap='gray', vmin=0.0, vmax=1.0, interpolation='nearest')
        ax5.set_title('2D Digit Projection')
        ax5.set_xticks([])
        ax5.set_yticks([])

        z_range = (
            f"calibrated {self.z_near:.4f}->{self.z_far:.4f}"
            if self.z_near is not None and self.z_far is not None
            else "relative trail range"
        )
        ax6.set_title('EasyOCR Digit Classifier')
        ax6.set_ylim(0.0, 1.0)
        ax6.set_xlim(-0.5, 9.5)
        ax6.set_xticks(range(10))
        ax6.set_ylabel('Probability')
        prob_bars = ax6.bar(range(10), np.zeros(10), color='#7c9cc8')
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
        if self.classifier is None:
            info_text.set_text(
                "Prediction: classifier not loaded\n"
                "Runner-up: --\n"
                f"Z: -- ({z_range})"
            )

        def init_plot():
            line_bx.set_data([], [])
            line_by.set_data([], [])
            line_bz.set_data([], [])
            image_artist.set_data(projection_image)
            return line_bx, line_by, line_bz, image_artist

        def update_plot(frame):
            data = self.get_data_copy()
            if not data:
                return line_bx, line_by, line_bz, image_artist

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

            # Update axis limits
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
                    ax.legend(loc='upper right')

            pose_x, pose_y, pose_z = self.extract_pose_trail(data)
            digit_image = self.pose_to_digit_image(pose_x, pose_y, pose_z)
            image_artist.set_data(digit_image)
            prediction_text = "Prediction: --"
            runner_up_text = "Runner-up: --"
            self._classifier_frame_skip += 1
            if self.classifier is not None and self._classifier_frame_skip % 3 == 0:
                try:
                    result = self.classifier.predict_smoothed(
                        digit_image.astype(np.float32),
                        self.prediction_history,
                    )
                    probabilities = result['probabilities']
                    top_two = np.argsort(probabilities)[-2:][::-1]
                    prediction_text = f"Prediction: {int(top_two[0])} ({probabilities[top_two[0]] * 100:.1f}%)"
                    runner_up_text = f"Runner-up:  {int(top_two[1])} ({probabilities[top_two[1]] * 100:.1f}%)"
                    for digit, bar in enumerate(prob_bars):
                        bar.set_height(float(probabilities[digit]))
                        bar.set_color('#2f5f9f' if digit == int(top_two[0]) else '#7c9cc8')
                except Exception as e:
                    prediction_text = f"Prediction failed: {e}"
                    runner_up_text = "Runner-up: --"

            if len(pose_z) > 0:
                finite_z = pose_z[np.isfinite(pose_z)]
                if len(finite_z) > 0:
                    info_text.set_text(
                        f"{prediction_text}\n"
                        f"{runner_up_text}\n"
                        f"Z: {finite_z[-1]:.5f} [{np.min(finite_z):.5f}, {np.max(finite_z):.5f}]"
                    )
            else:
                info_text.set_text(f"{prediction_text}\n{runner_up_text}\nZ: -- ({z_range})")

            ax4.clear()
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

            if len(pose_x) > 1:
                colors = np.linspace(0, 1, len(pose_x))
                scatter_3d = ax4.scatter(pose_x, pose_y, pose_z, c=colors, cmap='plasma', s=150, edgecolors='k', linewidths=0.6)
                ax4.plot(pose_x, pose_y, pose_z, color='#333333', alpha=0.65, linewidth=1.5)
                ax4.scatter(pose_x[-1:], pose_y[-1:], pose_z[-1:], c='red', s=120, label='Current position')
                ax4.legend(loc='best', fontsize='small')

            plt.tight_layout()
            return line_bx, line_by, line_bz, image_artist

        ani = FuncAnimation(fig, update_plot, init_func=init_plot, interval=100, blit=False)
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
            print("  s:   Stop current recording and save CSV + projected PNG")
            print("  q:   Quit program")
            print(f"Projection: X/Y plane, Z intensity, {self.image_size}x{self.image_size}px, last {self.trail_length} samples")
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
                       help='Directory for projected digit PNGs (default: digit_images)')
    parser.add_argument('--projection-extent', type=float, default=POSE_LIMIT,
                       help='Half-width of the X/Y projection area in pose units (default: 0.05)')
    parser.add_argument('--z-close-mode', choices=['min', 'max', 'abs-min'], default='max',
                       help='How Z maps to closeness/darkness: max means larger Z is darker (default)')
    parser.add_argument('--z-near', type=float, default=None,
                       help='Z value for darkest stroke. Use with --z-far for fixed height calibration')
    parser.add_argument('--z-far', type=float, default=None,
                       help='Z value for lightest stroke. Use with --z-near for fixed height calibration')
    parser.add_argument('--no-classifier', action='store_true',
                       help='Skip loading the real-time EasyOCR digit classifier')
    parser.add_argument('--classifier-gpu', action='store_true',
                       help='Use GPU for EasyOCR if available')

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
    print(f"Digit Classifier: {'disabled' if args.no_classifier else 'EasyOCR'}")
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
    )
    reader.run(csv_filename=None if args.no_csv else args.csv, baudrate=args.baudrate)


if __name__ == "__main__":
    main()
