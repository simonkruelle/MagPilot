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
import numpy as np
import signal
import sys
from collections import deque

class MagnetometerReader:
    def __init__(self):
        self.serial_port = None
        self.is_running = False
        self.read_thread = None
        self.data_lock = threading.Lock()
        self.data_buffer = deque(maxlen=1000)  # Keep last 1000 readings for plotting
        self.csv_file = None
        self.csv_writer = None

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
            print(f"{i+1}. {port.device} - {port.description}")

        return ports

    def choose_port(self, ports):
        """Let user choose a serial port."""
        while True:
            try:
                choice = input("Choose a port (number): ").strip()
                port_index = int(choice) - 1
                if 0 <= port_index < len(ports):
                    return ports[port_index].device
                else:
                    print("Invalid choice. Please try again.")
            except ValueError:
                print("Please enter a valid number.")

    def open_port(self, port, baudrate=921600):
        """Open the serial port."""
        try:
            self.serial_port = serial.Serial(
                port=port,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1
            )
            print(f"Opened serial port {port} at {baudrate} baud")
            return True
        except serial.SerialException as e:
            print(f"Error opening serial port: {e}")
            return False

    def start_csv_logging(self, filename="magnetometer_data.csv"):
        """Initialize CSV file for data logging."""
        try:
            self.csv_file = open(filename, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)

            # Write header
            header = []
            # 16 sensors * 3 axes = 48 magnetic field components
            for sensor in range(1, 17):
                header.extend([f'Bx{sensor}', f'By{sensor}', f'Bz{sensor}'])
            # 6 pose components: x, y, z, mx, my, mz
            header.extend(['x', 'y', 'z', 'mx', 'my', 'mz'])

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
        """
        if len(data) != 218:  # 1 header + 216 data + 1 tail
            return None

        if data[0] != 0xAA or data[-1] != 0xBB:
            return None

        try:
            # Unpack 54 floats (little-endian)
            floats = struct.unpack('<54f', data[1:-1])

            # First 48 are magnetic field data (16 sensors * 3 axes)
            mag_data = floats[:48]
            # Last 6 are pose data
            pose_data = floats[48:]

            return mag_data, pose_data
        except struct.error:
            return None

    def read_serial_data(self):
        """Thread function to read serial data at 100 Hz."""
        buffer = bytearray()
        target_interval = 1.0 / 100.0  # 100 Hz

        while self.is_running:
            start_time = time.time()

            try:
                # Read available bytes
                if self.serial_port.in_waiting > 0:
                    new_data = self.serial_port.read(self.serial_port.in_waiting)
                    buffer.extend(new_data)

                # Process buffer for complete packets
                while len(buffer) >= 218:  # Minimum packet size
                    # Look for header
                    header_pos = buffer.find(b'\xAA')
                    if header_pos == -1:
                        # No header found, clear buffer
                        buffer.clear()
                        break

                    # Remove data before header
                    if header_pos > 0:
                        del buffer[:header_pos]

                    # Check if we have enough data for a complete packet
                    if len(buffer) >= 218:
                        packet_data = buffer[:218]

                        # Parse the packet
                        parsed = self.parse_packet(packet_data)
                        if parsed:
                            mag_data, pose_data = parsed

                            # Prepare row data for CSV
                            row_data = list(mag_data) + list(pose_data)

                            # Thread-safe data storage
                            with self.data_lock:
                                self.data_buffer.append(row_data)

                                # Write to CSV
                                if self.csv_writer:
                                    self.csv_writer.writerow(row_data)

                            # Print decoded data (first few values for brevity)
                            print(f"Mag[0-2]: {mag_data[0]:.6f}, {mag_data[1]:.6f}, {mag_data[2]:.6f} | "
                                  f"Pose: {pose_data[0]:.6f}, {pose_data[1]:.6f}, {pose_data[2]:.6f}")

                        # Remove processed packet from buffer
                        del buffer[:218]
                    else:
                        break

            except serial.SerialException as e:
                print(f"Serial read error: {e}")
                break
            except Exception as e:
                print(f"Unexpected error in read thread: {e}")
                break

            # Maintain 100 Hz timing
            elapsed = time.time() - start_time
            if elapsed < target_interval:
                time.sleep(target_interval - elapsed)

    def start_reading(self):
        """Start the reading thread."""
        if not self.serial_port or not self.serial_port.is_open:
            print("Serial port not open!")
            return False

        self.is_running = True
        self.read_thread = threading.Thread(target=self.read_serial_data, daemon=True)
        self.read_thread.start()
        print("Started reading thread at 100 Hz")
        return True

    def get_data_for_plotting(self):
        """Get a copy of the data buffer for plotting (thread-safe)."""
        with self.data_lock:
            return list(self.data_buffer)

    def plot_data(self):
        """Create real-time plot of the magnetometer data."""
        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(12, 8))
        fig.suptitle('Real-time Magnetometer Data')

        # Initialize empty plots
        lines = []
        for ax in [ax1, ax2, ax3, ax4]:
            line, = ax.plot([], [], 'b-')
            lines.append(line)
            ax.set_xlim(0, 100)  # Show last 100 readings
            ax.grid(True)

        ax1.set_title('Magnetic Field X (Sensor 1)')
        ax1.set_ylabel('Bx1')

        ax2.set_title('Magnetic Field Y (Sensor 1)')
        ax2.set_ylabel('By1')

        ax3.set_title('Magnetic Field Z (Sensor 1)')
        ax3.set_ylabel('Bz1')

        ax4.set_title('Position X')
        ax4.set_ylabel('X Position')

        def update_plot(frame):
            data = self.get_data_for_plotting()
            if not data:
                return lines

            # Convert to numpy array for easier manipulation
            data_array = np.array(data)

            # Update x-axis (time)
            x_data = np.arange(len(data_array))

            # Update each subplot
            if len(data_array) > 0:
                # Bx1 (index 0)
                lines[0].set_data(x_data, data_array[:, 0])
                # By1 (index 1)
                lines[1].set_data(x_data, data_array[:, 1])
                # Bz1 (index 2)
                lines[2].set_data(x_data, data_array[:, 2])
                # X position (index 48)
                lines[3].set_data(x_data, data_array[:, 48])

                # Adjust x-axis limits
                for ax in [ax1, ax2, ax3, ax4]:
                    ax.set_xlim(max(0, len(data_array) - 100), len(data_array))

                # Adjust y-axis limits dynamically
                for i, line in enumerate(lines):
                    if len(data_array) > 0:
                        y_data = data_array[:, [0, 1, 2, 48][i]]
                        y_min, y_max = np.min(y_data), np.max(y_data)
                        margin = (y_max - y_min) * 0.1 if y_max != y_min else 1.0
                        [ax1, ax2, ax3, ax4][i].set_ylim(y_min - margin, y_max + margin)

            return lines

        ani = FuncAnimation(fig, update_plot, interval=100, blit=True, cache_frame_data=False)
        plt.tight_layout()
        plt.show()

    def signal_handler(self, signum, frame):
        """Handle Ctrl+C for graceful shutdown."""
        print("\nShutting down...")
        self.stop()

    def stop(self):
        """Stop reading and close resources."""
        self.is_running = False

        if self.read_thread and self.read_thread.is_alive():
            self.read_thread.join(timeout=2.0)

        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            print("Serial port closed")

        if self.csv_file:
            self.csv_file.close()
            print("CSV file closed")

    def run(self):
        """Main function to run the magnetometer reader."""
        try:
            # List and choose port
            ports = self.list_ports()
            if not ports:
                return

            port = self.choose_port(ports)

            # Open serial port
            if not self.open_port(port):
                return

            # Start CSV logging
            if not self.start_csv_logging():
                return

            # Start reading thread
            if not self.start_reading():
                return

            print("Press Ctrl+C to stop...")

            # Start plotting in main thread
            self.plot_data()

        except KeyboardInterrupt:
            pass
        finally:
            self.stop()


def main():
    reader = MagnetometerReader()
    reader.run()


if __name__ == "__main__":
    main()