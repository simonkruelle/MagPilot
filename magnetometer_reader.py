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

class MagnetometerReader:
    def __init__(self, plot_sensor=1):
        self.serial_port = None
        self.is_running = False
        self.read_thread = None
        self.data_lock = threading.Lock()
        self.data_buffer = deque(maxlen=1000)  # Keep last 1000 readings for plotting
        self.csv_file = None
        self.csv_writer = None
        self.plot_sensor = plot_sensor  # Which sensor to plot (1-16)

        # Session recording
        self.recording_sessions = []  # List of (session_name, start_time, end_time, data_points)
        self.current_session = None
        self.session_data = []

        # Setup signal handler for graceful shutdown
        signal.signal(signal.SIGINT, self.signal_handler)

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
                timeout=1
            )
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
        print(f"DEBUG - parse_packet called with {len(data)} bytes")
        if len(data) != 218:  # 1 header + 216 data + 1 tail
            print(f"DEBUG - Wrong packet size: {len(data)}, expected 218")
            return None

        if data[0] != 0xAA or data[-1] != 0xBB:
            print(f"DEBUG - Wrong header/tail: header=0x{data[0]:02X} (expected 0xAA), tail=0x{data[-1]:02X} (expected 0xBB)")
            return None

        try:
            # Try little-endian first (most common)
            floats = struct.unpack('<54f', data[1:217])  # Skip header, include 216 bytes of float data
        except struct.error:
            try:
                # Try big-endian if little-endian fails
                floats = struct.unpack('>54f', data[1:217])
                print("DEBUG - Used big-endian byte order")
            except struct.error as e:
                print(f"DEBUG - Failed to unpack floats: {e}")
                return None

        # Split into magnetic field data (48 floats) and pose data (6 floats)
        mag_data = floats[:48]  # 16 sensors × 3 axes
        pose_data = floats[48:]  # 6 pose values

        return mag_data, pose_data

    def read_serial_data(self):
        """Thread function to read serial data at 100 Hz."""
        buffer = bytearray()
        target_hz = 100
        interval = 1.0 / target_hz

        print(f"Starting serial read thread at {target_hz} Hz")

        while self.is_running:
            start_time = time.time()

            if self.serial_port and self.serial_port.is_open:
                try:
                    # Read available data
                    data = self.serial_port.read(self.serial_port.in_waiting or 1)
                    if data:
                        buffer.extend(data)

                        # Look for complete packets
                        while len(buffer) >= 218:  # Minimum packet size
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
                            if len(buffer) >= 218:
                                # Check for end marker (0xBB at position 217)
                                if buffer[217] == 0xBB:
                                    # Extract packet
                                    packet_data = buffer[:218]

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

                                            # Write to CSV
                                            if self.csv_writer:
                                                self.csv_writer.writerow(row_data)

                                            # Store in current session if recording
                                            if self.current_session:
                                                self.session_data.append(row_data)

                                        # Print decoded data in a well-formatted way
                                        print(f"\n{'='*60}")
                                        print(f"PACKET RECEIVED - Sensor {self.plot_sensor} (Highlighted)")
                                        print(f"{'='*60}")

                                        # Show magnetic field data for all sensors
                                        print(f"MAGNETIC FIELD DATA (Bx, By, Bz per sensor):")
                                        for i in range(16):
                                            bx = mag_data[i*3]
                                            by = mag_data[i*3 + 1]
                                            bz = mag_data[i*3 + 2]
                                            marker = " <-- PLOTTING" if i+1 == self.plot_sensor else ""
                                            print(f"  Sensor {i+1:2d}: Bx={bx:>10.6f}, By={by:>10.6f}, Bz={bz:>10.6f}{marker}")

                                        # Show pose data
                                        print(f"\nPOSE DATA:")
                                        print(f"  Position:  x={pose_data[0]:>10.6f}, y={pose_data[1]:>10.6f}, z={pose_data[2]:>10.6f}")
                                        print(f"  Rotation: mx={pose_data[3]:>10.6f}, my={pose_data[4]:>10.6f}, mz={pose_data[5]:>10.6f}")

                                        # Show units reminder
                                        print(f"\nUNITS REMINDER:")
                                        print(f"  Magnetic field units depend on your sensor hardware.")
                                        print(f"  Typical ranges: Tesla (0.00001-0.0001), Gauss (0.0001-0.001), μTesla (10-100)")
                                        print(f"  Check your magnetometer datasheet for exact specifications.")

                                        # Show recording status
                                        if self.current_session:
                                            print(f"  📹 RECORDING: {self.current_session} ({len(self.session_data)} points)")

                                    else:
                                        print(f"DEBUG - Failed to parse packet: len={len(packet_data)}, header=0x{packet_data[0]:02X}, tail=0x{packet_data[-1]:02X}")

                                    # Remove processed packet from buffer
                                    del buffer[:218]
                                else:
                                    # End marker not found, remove start marker and continue
                                    del buffer[0]
                            else:
                                # Not enough data for complete packet, wait for more
                                break

                except serial.SerialException as e:
                    print(f"Serial read error: {e}")
                    time.sleep(0.1)

            # Maintain target frequency
            elapsed = time.time() - start_time
            if elapsed < interval:
                time.sleep(interval - elapsed)

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
        if key == '1':
            self.start_session('no_magnet')
        elif key == '2':
            self.start_session('magnet_still')
        elif key == '3':
            self.start_session('magnet_moving')
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
        print(f"\n🎬 STARTED RECORDING SESSION: '{session_name}'")
        print(f"Recording to main CSV file: {self.csv_file.name if self.csv_file else 'None'}")

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

    def get_data_copy(self):
        """Get a copy of the data buffer for plotting (thread-safe)."""
        with self.data_lock:
            return list(self.data_buffer)

    def plot_data(self):
        """Create real-time plot of Bx, By, Bz for the selected sensor."""
        fig = plt.figure(figsize=(12, 8))
        fig.suptitle(f'Real-time Magnetometer Data - Sensor {self.plot_sensor}', fontsize=14)

        # Create subplots: 3 2D plots and 1 3D plot
        ax1 = fig.add_subplot(2, 2, 1)
        ax2 = fig.add_subplot(2, 2, 2)
        ax3 = fig.add_subplot(2, 2, 3)
        ax4 = fig.add_subplot(2, 2, 4, projection='3d')

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

        # 3D sensor-grid field magnitude plot
        ax4.set_title('16-Sensor Grid Field Magnitude')
        ax4.set_xlabel('Grid X')
        ax4.set_ylabel('Grid Y')
        ax4.set_zlabel('Field magnitude')
        ax4.grid(True)

        sensor_grid_x = np.array([i % 4 for i in range(16)])
        sensor_grid_y = np.array([i // 4 for i in range(16)])

        # Initialize empty 3D scatter - will be updated in animation
        scatter_3d = ax4.scatter(sensor_grid_x, sensor_grid_y, np.zeros(16), c=np.zeros(16), cmap='viridis', s=60)

        def init_plot():
            line_bx.set_data([], [])
            line_by.set_data([], [])
            line_bz.set_data([], [])
            return line_bx, line_by, line_bz, scatter_3d

        def update_plot(frame):
            data = self.get_data_copy()
            if not data:
                return line_bx, line_by, line_bz, scatter_3d

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
                    ax.set_ylim(min(data_vals) * 1.1, max(data_vals) * 1.1)

            # Use the most recent full-row reading for the 16-sensor grid
            latest_row = data[-1]
            mag_data = latest_row[1:49]
            magnitudes = np.array([np.sqrt(mag_data[i*3]**2 + mag_data[i*3+1]**2 + mag_data[i*3+2]**2) for i in range(16)])

            ax4.clear()
            ax4.set_title('16-Sensor Grid Field Magnitude')
            ax4.set_xlabel('Grid X')
            ax4.set_ylabel('Grid Y')
            ax4.set_zlabel('Field magnitude')
            ax4.grid(True)

            if len(magnitudes) == 16:
                scatter_3d = ax4.scatter(sensor_grid_x, sensor_grid_y, magnitudes, c=magnitudes, cmap='viridis', s=80)
                ax4.plot_trisurf(sensor_grid_x, sensor_grid_y, magnitudes, cmap='viridis', alpha=0.5, linewidth=0.2)

            plt.tight_layout()
            return line_bx, line_by, line_bz, scatter_3d

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
            if not self.start_csv_logging(csv_filename):
                return

            # Start reading thread
            if not self.start_reading():
                return

            print("\n" + "="*60)
            print("MAGNETOMETER DATA ACQUISITION SYSTEM")
            print("="*60)
            print("Controls:")
            print("  1: Start recording 'no_magnet' session")
            print("  2: Start recording 'magnet_still' session")
            print("  3: Start recording 'magnet_moving' session")
            print("  s: Stop current recording session")
            print("  q: Quit program")
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

    args = parser.parse_args()

    print(f"Magnetometer Data Reader")
    print(f"Plotting Sensor: {args.sensor}")
    print(f"Baudrate: {args.baudrate}")
    print(f"CSV Output: {args.csv}")
    print("-" * 40)

    reader = MagnetometerReader(plot_sensor=args.sensor)
    reader.run(csv_filename=args.csv, baudrate=args.baudrate)


if __name__ == "__main__":
    main()