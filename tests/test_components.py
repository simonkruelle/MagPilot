#!/usr/bin/env python3
"""Tests for basic magnetometer reader components."""

import csv
import os as _os
import struct
import sys as _sys

import serial.tools.list_ports


_ROOT = _os.path.dirname(_os.path.dirname(_os.path.abspath(__file__)))
for _p in (_ROOT, _os.path.join(_ROOT, 'tools')):
    if _p not in _sys.path:
        _sys.path.insert(0, _p)


def test_port_listing():
    """Test serial port listing functionality."""
    ports = list(serial.tools.list_ports.comports())
    assert all(port.device for port in ports)


def test_packet_parsing():
    """Test packet parsing with sample data."""
    header = b'\xAA'
    tail = b'\xBB'
    floats = [float(i) * 0.1 for i in range(54)]
    data = struct.pack('<54f', *floats)
    packet = header + data + tail

    assert len(packet) == 218
    assert packet[0] == 0xAA
    assert packet[-1] == 0xBB

    parsed_floats = struct.unpack('<54f', packet[1:-1])
    assert len(parsed_floats) == 54
    assert all(abs(actual - expected) < 1e-6
               for actual, expected in zip(parsed_floats, floats))


def test_csv_writing(tmp_path):
    """Test CSV writing functionality."""
    test_file = tmp_path / 'test_output.csv'
    header = []
    for sensor in range(1, 17):
        header.extend([f'Bx{sensor}', f'By{sensor}', f'Bz{sensor}'])
    header.extend(['x', 'y', 'z', 'mx', 'my', 'mz'])
    sample_data = [i * 0.01 for i in range(54)]

    with test_file.open('w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(header)
        writer.writerow(sample_data)

    with test_file.open(newline='') as csvfile:
        rows = list(csv.reader(csvfile))

    assert rows[0] == header
    assert [float(value) for value in rows[1]] == sample_data
