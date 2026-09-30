#!/usr/bin/env python3
"""Record magnet-tracking error against known stylus placements.

The operator places the stylus at each target of a grid (x/y position,
height above the sensors, and dipole tilt/azimuth, usually set with a printed
jig), presses Enter, and the script captures the board's pose estimate for a
short window. Every target gets a CSV with the raw packets plus the ground
truth, a JSON sidecar, and one row in summary.csv with the mean estimate and
its errors. The run layout and manifest follow magnetometer_reader.py:

    <output_dir>/manifest.json
    <output_dir>/raw/<run_id>_raw.csv
    <output_dir>/samples/<label>/<run_id>_<label>_rep001_<timestamp>.{csv,json}
    <output_dir>/summary/<run_id>_summary.csv

Board frame: x/y in metres from the board centre, as reported in Pose_x/y;
z is the physical magnet height above the sensor plane (the recorder's
calibrated height). Tilt is measured from the board normal, azimuth follows
colmag.control_mapping.magnet_angles_degrees (atan2(mx, my)).

Usage:
  python3 tools/record_tracking_error.py --port /dev/ttyACM0
  python3 tools/record_tracking_error.py --simulate --run-id sim_check
  python3 tools/record_tracking_error.py --dry-run
"""

import argparse
import csv
import json
import math
import os
import random
import re
import struct
import subprocess
import sys
import time
from datetime import datetime

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from colmag.control_mapping import (  # noqa: E402
    MAGNET_HEIGHT_MIN_M,
    MAGNET_HEIGHT_SENSOR_BIAS_M,
    calibrated_magnet_height,
    magnet_angles_degrees,
)

# Packet layout shared with magnetometer_reader.py (kept local so this tool
# does not pull in matplotlib or the classifier).
PACKET_SIZE = 218
PACKET_HEADER = 0xAA
PACKET_TAIL = 0xBB
PACKET_STRUCT = struct.Struct('<54f')

# Sensor board: 4x4 magnetometers on a 150 x 150 mm board, 35 mm apart
# (measured 30 Sep 2026: 105 mm between the centres of S1 and S13), so the
# sensors sit at +/-17.5 and +/-52.5 mm. The default grid uses half the pitch
# so it alternates between points above sensors and points between them, out
# to the outer sensors.
BOARD_SIZE_MM = 150.0
SENSOR_GRID = 4
SENSOR_PITCH_MM = 35.0

TRACKING_MANIFEST_SCHEMA_VERSION = 1
TRACKING_TASK = 'magnet_tracking_error'

DEFAULT_GRID_SPACING_MM = SENSOR_PITCH_MM / 2.0
DEFAULT_GRID_EXTENT_MM = SENSOR_PITCH_MM * (SENSOR_GRID - 1) / 2.0
# Heights are the gap between the sensor plane and the surface the stylus rests
# on (the 5 mm foam board is the current minimum). The planned test heights run
# from near-contact to beyond the tracking range. The magnet's centre sits
# --magnet-offset-mm above that surface, which is what the board estimates.
DEFAULT_HEIGHTS_MM = (10.0, 50.0, 100.0, 150.0, 200.0)
DEFAULT_ORIENTATIONS = ((0.0, 0.0),)

GT_COLUMNS = [
    'Target_index', 'GT_x', 'GT_y', 'GT_z', 'GT_tilt_deg', 'GT_azimuth_deg',
    'Est_z_calibrated',
]
SUMMARY_COLUMNS = [
    'target_index', 'label', 'status',
    'gt_x', 'gt_y', 'gt_z', 'gt_tilt_deg', 'gt_azimuth_deg',
    'n_samples',
    'est_x', 'est_y', 'est_z', 'est_tilt_deg', 'est_azimuth_deg',
    'std_x', 'std_y', 'std_z',
    'err_x', 'err_y', 'err_z', 'err_xy', 'err_xyz',
    'err_angle_deg', 'err_axis_deg',
]


# --------------------------------------------------------------------------
# Shared helpers (same conventions as MagnetometerReader)
# --------------------------------------------------------------------------

def csv_header():
    """Return the recorder's raw row header (timestamp, 16x3 field, pose)."""
    header = ['timestamp']
    for i in range(16):
        header.extend([f'Sensor{i+1}_Bx', f'Sensor{i+1}_By', f'Sensor{i+1}_Bz'])
    header.extend(['Pose_x', 'Pose_y', 'Pose_z', 'Pose_mx', 'Pose_my', 'Pose_mz'])
    return header


def sanitize_path_component(value):
    """Make a user/run label safe as a single filename component."""
    safe = re.sub(r'[^A-Za-z0-9_.-]+', '_', str(value).strip())
    safe = safe.strip('._-')
    return safe or 'unnamed'


def collect_git_metadata():
    """Collect git branch/commit/dirty metadata for reproducibility."""
    git_info = {}
    try:
        commands = {
            'commit': ['git', 'log', '-1', '--format=%H', '--no-show-signature'],
            'branch': ['git', 'branch', '--show-current'],
            'status': ['git', 'status', '--porcelain'],
        }
        for key, command in commands.items():
            result = subprocess.run(
                command, capture_output=True, text=True, timeout=3, cwd=_ROOT,
            )
            if result.returncode == 0:
                if key == 'status':
                    git_info['dirty'] = len(result.stdout.strip()) > 0
                else:
                    git_info[key] = result.stdout.strip()
    except Exception:
        git_info['error'] = 'git metadata unavailable'
    return git_info


def parse_packet(data):
    """Return (mag_data, pose_data) from one framed packet, or None."""
    if len(data) != PACKET_SIZE:
        return None
    if data[0] != PACKET_HEADER or data[-1] != PACKET_TAIL:
        return None
    try:
        floats = PACKET_STRUCT.unpack(bytes(data[1:PACKET_SIZE - 1]))
    except struct.error:
        return None
    return floats[:48], floats[48:]


# --------------------------------------------------------------------------
# Geometry
# --------------------------------------------------------------------------

def direction_from_angles(tilt_deg, azimuth_deg):
    """Unit dipole vector for tilt from +z and azimuth atan2(mx, my)."""
    tilt = math.radians(tilt_deg)
    azimuth = math.radians(azimuth_deg)
    return (
        math.sin(tilt) * math.sin(azimuth),
        math.sin(tilt) * math.cos(azimuth),
        math.cos(tilt),
    )


def angle_between_deg(a, b):
    """Angle between two vectors in degrees (NaN if either is degenerate)."""
    na = math.sqrt(sum(v * v for v in a))
    nb = math.sqrt(sum(v * v for v in b))
    if na <= 1e-12 or nb <= 1e-12:
        return float('nan')
    dot = sum(x * y for x, y in zip(a, b)) / (na * nb)
    return math.degrees(math.acos(max(-1.0, min(1.0, dot))))


def grid_axis(extent_mm, spacing_mm):
    """Symmetric grid coordinates in mm from -extent to +extent."""
    if spacing_mm <= 0:
        raise ValueError('grid spacing must be positive')
    steps = int(math.floor(extent_mm / spacing_mm + 1e-9))
    return [i * spacing_mm for i in range(-steps, steps + 1)]


def magnet_z_m(target):
    """Ground-truth magnet-centre height in metres (surface gap + offset)."""
    return (target['z_mm'] + target.get('magnet_offset_mm', 0.0)) / 1000.0


def target_label(target):
    """Folder/file label for one target, e.g. x-20_y+00_z15_t00_a000."""
    return (
        f"x{target['x_mm']:+04.0f}_y{target['y_mm']:+04.0f}"
        f"_z{target['z_mm']:03.0f}_t{target['tilt_deg']:02.0f}"
        f"_a{target['azimuth_deg'] % 360:03.0f}"
    ).replace('+', 'p').replace('-', 'm')


def build_targets(spacing_mm, extent_mm, heights_mm, orientations):
    """Ordered target list: jig setting (height, orientation) outermost, then a
    serpentine sweep over the x/y grid so consecutive moves stay short."""
    axis = grid_axis(extent_mm, spacing_mm)
    targets = []
    for z_mm in heights_mm:
        for tilt_deg, azimuth_deg in orientations:
            for row, y_mm in enumerate(reversed(axis)):
                xs = axis if row % 2 == 0 else list(reversed(axis))
                for x_mm in xs:
                    targets.append({
                        'x_mm': x_mm, 'y_mm': y_mm, 'z_mm': z_mm,
                        'tilt_deg': tilt_deg, 'azimuth_deg': azimuth_deg,
                    })
    for index, target in enumerate(targets):
        target['index'] = index
        target['label'] = target_label(target)
    return targets


def load_targets_csv(path):
    """Load targets from a CSV with x_mm,y_mm,z_mm[,tilt_deg,azimuth_deg]."""
    targets = []
    with open(path, 'r', newline='') as f:
        for row in csv.DictReader(f):
            targets.append({
                'x_mm': float(row['x_mm']),
                'y_mm': float(row['y_mm']),
                'z_mm': float(row['z_mm']),
                'tilt_deg': float(row.get('tilt_deg') or 0.0),
                'azimuth_deg': float(row.get('azimuth_deg') or 0.0),
            })
    for index, target in enumerate(targets):
        target['index'] = index
        target['label'] = target_label(target)
    return targets


def summarize_capture(target, rows, sensor_bias_m, height_min_m):
    """Mean pose estimate and error metrics for one captured target."""
    gt = (target['x_mm'] / 1000.0, target['y_mm'] / 1000.0, magnet_z_m(target))
    gt_dir = direction_from_angles(target['tilt_deg'], target['azimuth_deg'])
    summary = {
        'target_index': target['index'],
        'label': target['label'],
        'status': 'ok' if rows else 'no_samples',
        'gt_x': gt[0], 'gt_y': gt[1], 'gt_z': gt[2],
        'gt_tilt_deg': target['tilt_deg'],
        'gt_azimuth_deg': target['azimuth_deg'],
        'n_samples': len(rows),
    }
    if not rows:
        return summary

    xs = [row[49] for row in rows]
    ys = [row[50] for row in rows]
    zs = [calibrated_magnet_height(row[51], sensor_bias_m, height_min_m) for row in rows]
    n = len(rows)
    mean = lambda values: sum(values) / n  # noqa: E731
    std = lambda values, m: math.sqrt(sum((v - m) ** 2 for v in values) / n)  # noqa: E731
    est = (mean(xs), mean(ys), mean(zs))
    est_dir = (
        mean([row[52] for row in rows]),
        mean([row[53] for row in rows]),
        mean([row[54] for row in rows]),
    )
    try:
        est_tilt, est_azimuth = magnet_angles_degrees(*est_dir)
    except ValueError:
        est_tilt = est_azimuth = float('nan')
    err = [e - g for e, g in zip(est, gt)]
    angle_err = angle_between_deg(est_dir, gt_dir)
    summary.update({
        'est_x': est[0], 'est_y': est[1], 'est_z': est[2],
        'est_tilt_deg': est_tilt, 'est_azimuth_deg': est_azimuth,
        'std_x': std(xs, est[0]), 'std_y': std(ys, est[1]), 'std_z': std(zs, est[2]),
        'err_x': err[0], 'err_y': err[1], 'err_z': err[2],
        'err_xy': math.hypot(err[0], err[1]),
        'err_xyz': math.sqrt(sum(e * e for e in err)),
        'err_angle_deg': angle_err,
        # Polarity-free axis error, for when the magnet sits flipped in the jig.
        'err_axis_deg': min(angle_err, 180.0 - angle_err) if math.isfinite(angle_err) else angle_err,
    })
    return summary


# --------------------------------------------------------------------------
# Sample sources
# --------------------------------------------------------------------------

class SerialSource:
    """Blocking packet reader over the board's serial stream."""

    def __init__(self, port_name, baudrate):
        try:
            import serial
        except ImportError as exc:
            raise SystemExit(
                f"pyserial is required for hardware capture ({exc}). "
                "Install with `pip install -r requirements.txt` or use --simulate."
            )
        self.port = serial.Serial(port=port_name, baudrate=baudrate, timeout=0.05)
        self.port.reset_input_buffer()
        self.buffer = bytearray()

    def flush(self):
        self.port.reset_input_buffer()
        self.buffer.clear()

    def read_rows(self, duration_s):
        """Yield raw CSV rows for duration_s seconds."""
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            data = self.port.read(self.port.in_waiting or PACKET_SIZE)
            if data:
                self.buffer.extend(data)
            while len(self.buffer) >= PACKET_SIZE:
                start = self.buffer.find(bytes([PACKET_HEADER]))
                if start == -1:
                    self.buffer.clear()
                    break
                if start > 0:
                    del self.buffer[:start]
                if len(self.buffer) < PACKET_SIZE:
                    break
                if self.buffer[PACKET_SIZE - 1] != PACKET_TAIL:
                    del self.buffer[0]
                    continue
                parsed = parse_packet(self.buffer[:PACKET_SIZE])
                del self.buffer[:PACKET_SIZE]
                if parsed:
                    mag_data, pose_data = parsed
                    yield [datetime.now().isoformat(), *mag_data, *pose_data]

    def close(self):
        self.port.close()


class SimulatedSource:
    """Ground truth plus Gaussian noise, for exercising the pipeline offline."""

    def __init__(self, rate_hz=100.0, pos_noise_m=0.001, dir_noise=0.02,
                 sensor_bias_m=MAGNET_HEIGHT_SENSOR_BIAS_M, seed=0):
        self.rate_hz = rate_hz
        self.pos_noise_m = pos_noise_m
        self.dir_noise = dir_noise
        self.sensor_bias_m = sensor_bias_m
        self.rng = random.Random(seed)
        self.target = None

    def flush(self):
        pass

    def read_rows(self, duration_s):
        target = self.target
        gt_dir = direction_from_angles(target['tilt_deg'], target['azimuth_deg'])
        for _ in range(max(1, int(round(duration_s * self.rate_hz)))):
            noise = lambda s: self.rng.gauss(0.0, s)  # noqa: E731
            pose = [
                target['x_mm'] / 1000.0 + noise(self.pos_noise_m),
                target['y_mm'] / 1000.0 + noise(self.pos_noise_m),
                # Board reports biased raw Z; undo the recorder's calibration.
                magnet_z_m(target) + self.sensor_bias_m + noise(self.pos_noise_m),
                *[c + noise(self.dir_noise) for c in gt_dir],
            ]
            yield [datetime.now().isoformat(), *([0.0] * 48), *pose]

    def close(self):
        pass


# --------------------------------------------------------------------------
# Recorder
# --------------------------------------------------------------------------

class TrackingErrorRecorder:
    def __init__(self, output_dir, run_id, targets, settings, source,
                 input_source, raw_csv=True, prompt=input):
        self.output_dir = output_dir
        self.run_id = sanitize_path_component(run_id)
        self.targets = targets
        self.settings = settings
        self.source = source
        self.input_source = input_source
        self.prompt = prompt
        self.command_line = list(sys.argv)
        self.manifest_path = os.path.join(output_dir, 'manifest.json')
        self.raw_csv_path = (
            os.path.join(output_dir, 'raw', f'{self.run_id}_raw.csv') if raw_csv else None
        )
        self.summary_path = os.path.join(output_dir, 'summary', f'{self.run_id}_summary.csv')
        self.raw_file = None
        self.raw_writer = None

    # ---- layout / manifest ------------------------------------------------

    def rel(self, path):
        return os.path.relpath(path, self.output_dir) if path else None

    def prepare_layout(self):
        for sub in ('raw', 'samples', 'summary'):
            os.makedirs(os.path.join(self.output_dir, sub), exist_ok=True)
        self.write_manifest()

    def load_manifest(self):
        base = {
            'schema_version': TRACKING_MANIFEST_SCHEMA_VERSION,
            'created_at': datetime.now().isoformat(),
            'sessions': [],
        }
        if not os.path.exists(self.manifest_path):
            return base
        try:
            with open(self.manifest_path, 'r', encoding='utf-8') as f:
                manifest = json.load(f)
        except (IOError, json.JSONDecodeError):
            return base
        manifest.setdefault('sessions', [])
        return manifest

    def write_manifest(self, session_entry=None):
        manifest = self.load_manifest()
        if session_entry is not None:
            manifest['sessions'].append(session_entry)
        manifest.update({
            'schema_version': TRACKING_MANIFEST_SCHEMA_VERSION,
            'task': TRACKING_TASK,
            'updated_at': datetime.now().isoformat(),
            'run_id': self.run_id,
            'output_dir': os.path.abspath(self.output_dir),
            'command_line': self.command_line,
            'input_source': self.input_source,
            'git': collect_git_metadata(),
            'settings': self.settings,
            'target_count': len(self.targets),
            'raw_log': {
                'enabled': bool(self.raw_csv_path),
                'path': self.rel(self.raw_csv_path),
            },
            'summary': self.rel(self.summary_path),
        })
        temp_path = f'{self.manifest_path}.tmp'
        with open(temp_path, 'w', encoding='utf-8') as f:
            json.dump(manifest, f, indent=2)
            f.write('\n')
        os.replace(temp_path, self.manifest_path)

    def next_repetition(self, label):
        rep = 1
        for session in self.load_manifest().get('sessions', []):
            if session.get('run_id') == self.run_id and session.get('label') == label:
                rep = max(rep, int(session.get('repetition', 0)) + 1)
        return rep

    # ---- capture ------------------------------------------------------------

    def capture(self, target):
        if isinstance(self.source, SimulatedSource):
            self.source.target = target
        self.source.flush()
        for _ in self.source.read_rows(self.settings['settle_s']):
            pass
        rows = list(self.source.read_rows(self.settings['capture_s']))
        if self.raw_writer:
            self.raw_writer.writerows(rows)
            self.raw_file.flush()
        return rows

    def save_target(self, target, rows, started_at, ended_at):
        label = target['label']
        label_dir = os.path.join(self.output_dir, 'samples', label)
        os.makedirs(label_dir, exist_ok=True)
        repetition = self.next_repetition(label)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        basename = f'{self.run_id}_{label}_rep{repetition:03d}_{timestamp}'
        csv_path = os.path.join(label_dir, f'{basename}.csv')
        json_path = os.path.join(label_dir, f'{basename}.json')

        bias = self.settings['sensor_bias_m']
        height_min = self.settings['height_min_m']
        gt_values = [
            target['index'], target['x_mm'] / 1000.0, target['y_mm'] / 1000.0,
            magnet_z_m(target), target['tilt_deg'], target['azimuth_deg'],
        ]
        with open(csv_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(csv_header() + GT_COLUMNS)
            for row in rows:
                z_cal = calibrated_magnet_height(row[51], bias, height_min)
                writer.writerow(row + gt_values + [z_cal])

        summary = summarize_capture(target, rows, bias, height_min)
        metadata = {
            'task': TRACKING_TASK,
            'label': label,
            'run_id': self.run_id,
            'repetition': repetition,
            'basename': basename,
            'created_at': datetime.now().isoformat(),
            'started_at': started_at,
            'ended_at': ended_at,
            'sample_count': len(rows),
            'target': target,
            'summary': summary,
            'paths': {'csv': self.rel(csv_path), 'json': self.rel(json_path)},
            'input_source': self.input_source,
            'command_line': self.command_line,
            'settings': self.settings,
            'git': collect_git_metadata(),
        }
        with open(json_path, 'w', encoding='utf-8') as f:
            json.dump(metadata, f, indent=2)
            f.write('\n')

        self.write_manifest(session_entry={
            'session_name': label,
            'label': label,
            'run_id': self.run_id,
            'repetition': repetition,
            'basename': basename,
            'target_index': target['index'],
            'started_at': started_at,
            'ended_at': ended_at,
            'saved_at': datetime.now().isoformat(),
            'sample_count': len(rows),
            'paths': {'csv': self.rel(csv_path), 'json': self.rel(json_path)},
            'error': {k: summary.get(k) for k in ('err_xy', 'err_xyz', 'err_angle_deg')},
        })
        return summary

    def append_summary(self, summary):
        new_file = not os.path.exists(self.summary_path)
        with open(self.summary_path, 'a', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=SUMMARY_COLUMNS)
            if new_file:
                writer.writeheader()
            writer.writerow({k: summary.get(k, '') for k in SUMMARY_COLUMNS})

    @staticmethod
    def describe(target):
        return (
            f"x={target['x_mm']:+.2f} mm  y={target['y_mm']:+.2f} mm  "
            f"z={target['z_mm']:.0f} mm  tilt={target['tilt_deg']:.0f} deg  "
            f"azimuth={target['azimuth_deg']:.0f} deg"
        )

    def run(self, auto=False):
        self.prepare_layout()
        if self.raw_csv_path:
            self.raw_file = open(self.raw_csv_path, 'a', newline='')
            self.raw_writer = csv.writer(self.raw_file)
            if self.raw_file.tell() == 0:
                self.raw_writer.writerow(csv_header())

        print(f"Tracking-error run '{self.run_id}': {len(self.targets)} targets")
        print(f"Output: {self.output_dir}")
        print("Enter = capture, s = skip, r = redo previous, q = quit")
        index = 0
        previous_jig = None
        try:
            while index < len(self.targets):
                target = self.targets[index]
                jig = (target['z_mm'], target['tilt_deg'], target['azimuth_deg'])
                if jig != previous_jig:
                    print(
                        f"\n== Jig: height {target['z_mm']:.0f} mm, tilt "
                        f"{target['tilt_deg']:.0f} deg, azimuth {target['azimuth_deg']:.0f} deg =="
                    )
                    previous_jig = jig
                print(f"[{index + 1}/{len(self.targets)}] {self.describe(target)}")
                answer = '' if auto else self.prompt('  place stylus, then Enter > ').strip().lower()
                if answer == 'q':
                    break
                if answer == 's':
                    self.append_summary({
                        'target_index': target['index'], 'label': target['label'],
                        'status': 'skipped',
                    })
                    index += 1
                    continue
                if answer == 'r':
                    index = max(0, index - 1)
                    previous_jig = None
                    continue

                started_at = datetime.now().isoformat()
                rows = self.capture(target)
                ended_at = datetime.now().isoformat()
                summary = self.save_target(target, rows, started_at, ended_at)
                self.append_summary(summary)
                if summary['status'] == 'ok':
                    print(
                        f"  {summary['n_samples']} samples | xy err "
                        f"{summary['err_xy'] * 1000:.1f} mm | z err "
                        f"{summary['err_z'] * 1000:+.1f} mm | angle err "
                        f"{summary['err_angle_deg']:.1f} deg"
                    )
                else:
                    print("  no samples received; check the board and press r to retry")
                index += 1
        finally:
            if self.raw_file:
                self.raw_file.close()
            self.source.close()
            self.write_manifest()
        print(f"\nSummary: {self.summary_path}")
        print(f"Manifest: {self.manifest_path}")


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------

def parse_float_list(text):
    return tuple(float(v) for v in text.split(',') if v.strip())


def parse_orientations(text):
    """'0:0,20:0,20:90' -> ((0, 0), (20, 0), (20, 90)) as (tilt, azimuth)."""
    orientations = []
    for item in text.split(','):
        item = item.strip()
        if not item:
            continue
        tilt, _, azimuth = item.partition(':')
        orientations.append((float(tilt), float(azimuth or 0.0)))
    return tuple(orientations)


def choose_serial_port():
    try:
        import serial.tools.list_ports
    except ImportError:
        raise SystemExit('pyserial is required; pass --simulate to run without hardware')
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        raise SystemExit('No serial ports found; pass --port or --simulate')
    for i, port in enumerate(ports, start=1):
        print(f"  {i}: {port.device} ({port.description})")
    choice = input('Select port number: ').strip()
    return ports[int(choice) - 1].device


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--port', '-p', default=None, help='Serial port of the sensor board')
    parser.add_argument('--baudrate', '-b', type=int, default=921600)
    parser.add_argument('--simulate', action='store_true',
                        help='Use ground truth + noise instead of the board')
    parser.add_argument('--dry-run', action='store_true',
                        help='Create the layout and manifest, print targets, and exit')
    parser.add_argument('--auto', action='store_true',
                        help='Do not prompt between targets (useful with --simulate)')
    parser.add_argument('--output-dir', default=None,
                        help='Defaults to data/tracking_YYYY-MM-DD/')
    parser.add_argument('--run-id', default=None, help='Defaults to run_YYYYmmdd_HHMMSS')
    parser.add_argument('--grid-spacing-mm', type=float, default=DEFAULT_GRID_SPACING_MM,
                        help=f'Default: half the sensor pitch ({DEFAULT_GRID_SPACING_MM:g} mm)')
    parser.add_argument('--grid-extent-mm', type=float, default=DEFAULT_GRID_EXTENT_MM,
                        help='Grid runs from -extent to +extent on x and y; '
                             f'default: the outer sensor row ({DEFAULT_GRID_EXTENT_MM:g} mm)')
    parser.add_argument('--heights-mm', type=parse_float_list,
                        default=DEFAULT_HEIGHTS_MM,
                        help='Comma-separated surface heights above the sensors '
                             '(default 10,50,100,150,200; 5 mm foam board is the minimum)')
    parser.add_argument('--magnet-offset-mm', type=float, default=0.0,
                        help='Magnet centre above the surface the stylus rests on '
                             '(tip mode: tip to magnet centre; side mode: magnet radius)')
    parser.add_argument('--orientations', type=parse_orientations,
                        default=DEFAULT_ORIENTATIONS,
                        help='Comma-separated tilt:azimuth pairs in degrees, e.g. 0:0,20:0,20:90; '
                             'use 90:0,90:90 for the stylus lying sideways')
    parser.add_argument('--targets-csv', default=None,
                        help='Custom targets (x_mm,y_mm,z_mm[,tilt_deg,azimuth_deg]); overrides the grid')
    parser.add_argument('--magnet', default='unspecified',
                        help="Magnet in the stylus, e.g. '12x12mm_stack' or 'D6x10mm'; logged in the manifest")
    parser.add_argument('--spacer-mm', type=float, default=None,
                        help='Cover spacer height in use (e.g. 7, 10, 15, 20); logged in the manifest')
    parser.add_argument('--settle-s', type=float, default=0.5,
                        help='Seconds discarded after Enter before capture')
    parser.add_argument('--capture-s', type=float, default=1.0,
                        help='Seconds captured per target')
    parser.add_argument('--no-raw-csv', action='store_true',
                        help='Do not write the continuous raw log')
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    if args.output_dir is None:
        args.output_dir = f"data/tracking_{datetime.now().strftime('%Y-%m-%d')}"
    run_id = args.run_id or datetime.now().strftime('run_%Y%m%d_%H%M%S')

    if args.targets_csv:
        targets = load_targets_csv(args.targets_csv)
    else:
        targets = build_targets(
            args.grid_spacing_mm, args.grid_extent_mm, args.heights_mm, args.orientations,
        )
    for target in targets:
        target['magnet_offset_mm'] = args.magnet_offset_mm
    half_board = BOARD_SIZE_MM / 2.0
    if any(max(abs(t['x_mm']), abs(t['y_mm'])) > half_board for t in targets):
        print(f"Warning: some targets lie outside the +/-{half_board:.0f} mm board")

    settings = {
        'grid_spacing_mm': None if args.targets_csv else args.grid_spacing_mm,
        'grid_extent_mm': None if args.targets_csv else args.grid_extent_mm,
        'magnet_offset_mm': args.magnet_offset_mm,
        'heights_mm': sorted({t['z_mm'] for t in targets}),
        'orientations_deg': sorted({(t['tilt_deg'], t['azimuth_deg']) for t in targets}),
        'targets_csv': args.targets_csv,
        'board_size_mm': BOARD_SIZE_MM,
        'sensor_grid': [SENSOR_GRID, SENSOR_GRID],
        'sensor_pitch_mm': SENSOR_PITCH_MM,
        'magnet': args.magnet,
        'spacer_mm': args.spacer_mm,
        'settle_s': args.settle_s,
        'capture_s': args.capture_s,
        'sensor_bias_m': MAGNET_HEIGHT_SENSOR_BIAS_M,
        'height_min_m': MAGNET_HEIGHT_MIN_M,
        'frame': ('board centre origin, metres; GT_z = surface height + magnet offset, '
                  'compared with the calibrated height estimate'),
    }

    input_source = 'simulated' if args.simulate else 'serial'
    if args.dry_run:
        source = SimulatedSource()
    elif args.simulate:
        source = SimulatedSource()
    else:
        source = SerialSource(args.port or choose_serial_port(), args.baudrate)

    recorder = TrackingErrorRecorder(
        output_dir=args.output_dir,
        run_id=run_id,
        targets=targets,
        settings=settings,
        source=source,
        input_source=input_source,
        raw_csv=not args.no_raw_csv and not args.dry_run,
    )
    if args.dry_run:
        recorder.prepare_layout()
        for target in targets:
            print(f"[{target['index'] + 1:3d}] {recorder.describe(target)}")
        print(f"Dry run: {len(targets)} targets, manifest at {recorder.manifest_path}")
        return 0
    recorder.run(auto=args.auto)
    return 0


if __name__ == '__main__':
    sys.exit(main())
