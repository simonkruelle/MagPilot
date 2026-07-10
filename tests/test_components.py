#!/usr/bin/env python3
"""
Test script for magnetometer reader components
"""

import os as _os, sys as _sys
_ROOT = _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))
for _p in (_ROOT, _os.path.join(_ROOT, 'tools')):
    if _p not in _sys.path:
        _sys.path.insert(0, _p)

import serial.tools.list_ports
import struct
import csv
import os

def test_port_listing():
    """Test serial port listing functionality."""
    print("Testing serial port listing...")
    ports = serial.tools.list_ports.comports()
    print(f"Found {len(ports)} ports:")
    for i, port in enumerate(ports):
        print(f"  {i+1}. {port.device} - {port.description}")
    return ports

def test_packet_parsing():
    """Test packet parsing with sample data."""
    print("\nTesting packet parsing...")

    # Create sample packet: header + 54 floats + tail
    header = b'\xAA'
    tail = b'\xBB'

    # Generate 54 sample floats
    floats = []
    for i in range(54):
        floats.append(float(i) * 0.1)  # 0.0, 0.1, 0.2, ..., 5.3

    # Pack floats as little-endian
    data = struct.pack('<54f', *floats)

    # Create complete packet
    packet = header + data + tail

    print(f"Packet size: {len(packet)} bytes (expected: 218)")

    # Test parsing
    if len(packet) != 218:
        print("ERROR: Wrong packet size!")
        return False

    if packet[0] != 0xAA or packet[-1] != 0xBB:
        print("ERROR: Wrong header/tail!")
        return False

    try:
        parsed_floats = struct.unpack('<54f', packet[1:-1])
        print(f"Successfully parsed {len(parsed_floats)} floats")
        print(f"First 5 floats: {parsed_floats[:5]}")
        print(f"Last 5 floats: {parsed_floats[-5:]}")

        # Check values
        expected = [i * 0.1 for i in range(54)]
        if all(abs(a - b) < 1e-6 for a, b in zip(parsed_floats, expected)):
            print("All float values match!")
            return True
        else:
            print("ERROR: Float values don't match!")
            return False

    except struct.error as e:
        print(f"ERROR parsing floats: {e}")
        return False

def test_csv_writing():
    """Test CSV writing functionality."""
    print("\nTesting CSV writing...")

    test_file = "test_output.csv"
    try:
        with open(test_file, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)

            # Write header
            header = []
            for sensor in range(1, 17):
                header.extend([f'Bx{sensor}', f'By{sensor}', f'Bz{sensor}'])
            header.extend(['x', 'y', 'z', 'mx', 'my', 'mz'])
            writer.writerow(header)

            # Write sample data row
            sample_data = []
            for i in range(54):  # 48 mag + 6 pose
                sample_data.append(i * 0.01)
            writer.writerow(sample_data)

        print(f"Successfully wrote to {test_file}")

        # Check file contents
        with open(test_file, 'r') as f:
            lines = f.readlines()
            print(f"Header: {lines[0].strip()}")
            print(f"Data: {lines[1].strip()}")

        # Clean up
        os.remove(test_file)
        print("Test file cleaned up")
        return True

    except Exception as e:
        print(f"ERROR in CSV test: {e}")
        return False

def main():
    """Run all tests."""
    print("Running magnetometer reader component tests...\n")

    tests = [
        ("Port Listing", test_port_listing),
        ("Packet Parsing", test_packet_parsing),
        ("CSV Writing", test_csv_writing),
    ]

    results = []
    for name, test_func in tests:
        try:
            result = test_func()
            results.append((name, result))
        except Exception as e:
            print(f"ERROR in {name}: {e}")
            results.append((name, False))

    print("\n" + "="*50)
    print("TEST RESULTS:")
    for name, passed in results:
        status = "PASS" if passed else "FAIL"
        print(f"  {name}: {status}")

    all_passed = all(passed for _, passed in results)
    print(f"\nOverall: {'ALL TESTS PASSED' if all_passed else 'SOME TESTS FAILED'}")

if __name__ == "__main__":
    main()