#!/usr/bin/env python3
"""Probe the MagPilot sensor board and report what its firmware does.

The board (16 MLX90393 magnetometers, 4x4 on a 150 x 150 mm board) streams
218-byte packets over USB serial, but the firmware source is not available.
This script walks you through five steps, about one minute of recording
(stylus far away, still, sweeping, pressed onto one sensor, then a port
close/reopen in two parts, 5a and 5b), and answers: which board it is, the
real sample rate, whether host timestamps can be trusted, the field units,
whether a baseline is subtracted, whether opening the port resets or
recalibrates the board, whether sensors saturate, and what the pose means.

READ-ONLY: the script never writes to the serial port and never sets DTR, RTS
or break itself. It opens the port with pyserial's defaults, like
magnetometer_reader.py, so the operating system raises DTR/RTS on open as it
always does; the effect of that is one of the things being measured. Before
each step on a port that stays open it drops the bytes that piled up on this
computer while you read the prompt (the same host-side flush pyserial does on
every open; nothing is sent to the board).

Each run writes one folder named by local time (data_collection/ is gitignored):

    <output_dir>/<YYYYmmdd_HHMMSS>/meta.json         ports, open times, steps
    <output_dir>/<YYYYmmdd_HHMMSS>/<step>.bin        every raw byte received
    <output_dir>/<YYYYmmdd_HHMMSS>/<step>_reads.csv  t_ns,nbytes of each read
    <output_dir>/<YYYYmmdd_HHMMSS>/report.md         answers in plain English
    <output_dir>/<YYYYmmdd_HHMMSS>/report.json       all numbers

Close the launcher and magnetometer_reader first: two programs reading one port
each get only part of the stream.

Usage inside the container (-it is needed for the Enter prompts; use
/host/dev/ttyACM0 if the board was plugged in after the container started):
  docker exec -it colmag_simon bash -lc \\
      'cd /colmag && python3 tools/probe_board.py --port /dev/ttyACM0'
Usage on the host (needs pyserial and permission to open the port):
  python3 tools/probe_board.py                  # uses the only ttyACM/ttyUSB port
  python3 tools/probe_board.py --listen-only    # 10 s check that the board streams
  python3 tools/probe_board.py --replay data_collection/board_probe/20260929_201500
"""

import argparse
import bisect
import csv
import glob
import json
import math
import os
import platform
import re
import statistics
import struct
import sys
import time
from collections import Counter
from datetime import datetime

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from colmag.control_mapping import (  # noqa: E402
    MAGNET_HEIGHT_SENSOR_BIAS_M,
    calibrated_magnet_height,
)

TOOL_VERSION = '1.2'

# Packet layout shared with magnetometer_reader.py.
PACKET_SIZE = 218
PACKET_HEADER = 0xAA
PACKET_TAIL = 0xBB
PACKET_STRUCT = struct.Struct('<54f')
N_SENSORS = 16
N_FIELD_CHANNELS = 3 * N_SENSORS  # Sensor1_Bx .. Sensor16_Bz; the pose follows
POSE_LIMIT_M = 0.05             # magnetometer_reader.py image window (+/-50 mm)

# Step durations in seconds: long enough for stable means, short enough to hold still.
FAR_S = 10.0
STILL_S = 5.0
SWEEP_S = 20.0
CLOSEST_S = 10.0
REOPEN_MAGNET_S = 5.0
REOPEN_FAR_S = 10.0
READ_TIMEOUT_S = 0.05           # short reads keep the countdown and Ctrl-C responsive

# Thresholds (each is a judgement call; the report always shows the numbers).
TEXT_MIN_CHARS = 6              # shorter printable runs are usually just float bytes
TEXT_MAX_CHARS = 500            # keep the report readable
MIN_PACKETS = 20                # fewer in the far step: random bytes also hold a few
MAX_GARBAGE_FRACTION = 0.1      # more skipped bytes: wrong baud rate or a second reader
GAP_EDGES_MS = (1, 5, 20, 50, 100)
BATCHED_FRACTION = 0.2          # more <1 ms gaps than this: timestamps show USB batching
BASELINE_NOISE_RATIO = 5.0      # far |B| under 5x the noise: Earth's field is not in the data
CLIP_MIN_RUN = 3                # an extreme repeated this often in a row looks pinned
CLIP_MIN_FRACTION = 0.5         # ...and is at least half the largest value seen anywhere
CLIP_MIN_NOISE = 20.0           # ...and far above the far-step noise
CLIP_MIN_RANGE = 0.25           # ...while its sensor moved (still magnets are not clipping)
WRAP_MIN_FRACTION = 0.5         # sign-flip jump larger than half the channel's range
FLAT_TOL = 0.01                 # a flat top: within 1 % of the channel's largest |value|
FLAT_MIN_RUN = 4                # ...for this many samples in a row
FLAT_OTHER_CHANGE = 0.2         # ...while another axis of that sensor changed by 20 % of it
FULL_SCALE_TOL = 0.02           # within 2 % of a full scale: pinned at the limit
POSE_MIN_SWEEP_M = 0.03         # sweep x or y range under 3 cm: pose does not follow the stylus
ZERO_MOMENT = 1e-9              # |m| below this: the moment is not computed
UNIT_MOMENT_TOL = 0.05          # |m| within 5 % of 1 is a unit vector
STEADY_SPREAD = 0.02            # |m| relative spread under 2 % is constant
FITTED_SPREAD = 0.10            # over 10 % means the strength is fitted (or noisy)
OPEN_SHIFT_SIGMA = 5.0          # a channel moved at reopen if it shifted by more than 5 noise
OPEN_SHIFT_FRACTION = 0.25      # ...and at least a quarter of channels did
RESTART_DELAY_S = 0.5           # first packet later than this after open suggests a reboot
RESTART_TEXT_S = 3.0            # firmware text this soon after open suggests a boot banner
PAUSE_MEAN_GAPS = 10            # a pause is a gap over RESTART_DELAY_S and 10x the mean gap

# (unit, low, high, factor to uT): wide windows around Earth's 25-65 uT (NOAA),
# widened for steel furniture and the board's own electronics.
UNIT_WINDOWS = (
    ('tesla', 1.5e-5, 1.0e-4, 1e6), ('mT', 0.015, 0.1, 1e3),
    ('gauss', 0.15, 1.0, 100.0), ('uT', 15.0, 100.0, 1.0),
    ('mG', 150.0, 1000.0, 0.1), ('nT', 15000.0, 100000.0, 1e-3),
)
# MLX90393 full scale in uT (sensitivity x code range, datasheet Rev 012, HALLCONF 0xC).
FULL_SCALE_UT = (
    ('GAIN_SEL 7, RES 0 (Adafruit default)', 4915, 7930),   # 32767 x 0.150 / 0.242
    ('GAIN_SEL 5, RES 0', 8204, 13216),
    ('GAIN_SEL 0, RES 0', 24612, 39649),
    ('GAIN_SEL 0, RES 1', 49225, 79299),
    ('GAIN_SEL 0, RES 2/3 (largest)', 66098, 106480),
)
HALL_SATURATION_UT = 50000      # datasheet BSAT onset; readings above are not trustworthy

# MLX90393 uT per count at RES 0 for GAIN_SEL 0..7 (datasheet Rev 012, HALLCONF 0xC);
# each RES step doubles it. MLX_RES_COUNTS is the output range in counts per RES.
MLX_UT_PER_COUNT_XY = (0.751, 0.601, 0.451, 0.376, 0.300, 0.250, 0.200, 0.150)
MLX_UT_PER_COUNT_Z = (1.210, 0.968, 0.726, 0.605, 0.484, 0.403, 0.323, 0.242)
MLX_RES_COUNTS = (32768, 32768, 22000, 11000)
STEP_UNITS = (('uT', 1.0), ('gauss', 0.01), ('mT', 0.001), ('tesla', 1e-6))  # value per uT
STEP_TOL = 0.005                # a value step within 0.5 % of a table entry matches it
STEP_MAX_ABS = 1000.0           # only small readings: float32 keeps their steps exact
STEP_MIN_CHANNELS = 8           # this many channels of an axis must show the same step
POSE_FROZEN_M = 0.005           # pose range under 5 mm: the fit is not running (no magnet)

USB_VENDORS = {
    0x2341: 'Arduino', 0x2A03: 'Arduino', 0x16C0: 'Teensy (PJRC)',
    0x303A: 'Espressif native USB (ESP32-S2/S3/C3)',
    0x10C4: 'Silicon Labs CP210x USB-UART bridge', 0x1A86: 'WCH CH340 USB-UART bridge',
    0x0403: 'FTDI USB-UART bridge', 0x0483: 'STMicroelectronics (STM32)',
    0x2E8A: 'Raspberry Pi RP2040', 0x239A: 'Adafruit', 0x1B4F: 'SparkFun',
}
BRIDGE_VIDS = (0x10C4, 0x1A86, 0x0403)

# (label, name, port action, seconds, instruction). 'open' and 'reopen' open the
# port just before recording; 'keep' records on the port that is already open.
STEPS = (
    ('1', 'far', 'open', FAR_S,
     'Put the stylus at least 1 m away from the board.'),
    ('2', 'still', 'keep', STILL_S,
     'Hold the stylus upright and still on the cover over the centre of the board.'),
    ('3', 'sweep', 'keep', SWEEP_S,
     'Move the stylus slowly over the whole writing area, touching the cover, '
     'and tilt it now and then.'),
    ('4', 'closest', 'keep', CLOSEST_S,
     'Press the stylus onto the cover right above one sensor, '
     'then tilt it slowly in all directions.'),
    ('5a', 'reopen_magnet', 'reopen', REOPEN_MAGNET_S,
     'The port is closed. Put the stylus upright on the cover over the centre '
     'and leave it there.'),
    ('5b', 'reopen_far', 'keep', REOPEN_FAR_S,
     'Now take the stylus at least 1 m away.'),
)
MAGNET_STEPS = ('still', 'sweep', 'closest', 'reopen_magnet')

CHANNELS = ['Sensor%d_%s' % (i + 1, axis) for i in range(N_SENSORS) for axis in ('Bx', 'By', 'Bz')]
CHANNELS += ['Pose_x', 'Pose_y', 'Pose_z', 'Pose_mx', 'Pose_my', 'Pose_mz']
POSE = {name: N_FIELD_CHANNELS + i for i, name in enumerate(('x', 'y', 'z', 'mx', 'my', 'mz'))}

NOT_MEASURED = 'not measured: run the full probe (without --listen-only)'
WRONG_PORT_HINT = ('wrong port or baud rate, or another program is reading the port '
                   '(close the launcher / magnetometer_reader first)')


# --------------------------------------------------------------------------
# Small helpers
# --------------------------------------------------------------------------

def finite(values):
    return [v for v in values if math.isfinite(v)]


def mean_std(values):
    """Mean and population std of the finite values, or (None, None)."""
    good = finite(values)
    if not good:
        return None, None
    return statistics.fmean(good), statistics.pstdev(good)


def column_means(rows):
    return [mean_std([row[c] for row in rows])[0] for c in range(N_FIELD_CHANNELS)]


def fmt(value, digits=4):
    """Short number for the report: 'n/a' for None, whole numbers from 1000 up."""
    if value is None:
        return 'n/a'
    if 1000 <= abs(value) < 1e7:
        return '%.0f' % value
    return '%.*g' % (digits, value)


def read_json(path):
    with open(path, encoding='utf-8') as handle:
        return json.load(handle)


def write_json(path, payload):
    """Atomic JSON write (write .tmp, then replace), like record_tracking_error.py."""
    with open(path + '.tmp', 'w', encoding='utf-8') as handle:
        json.dump(payload, handle, indent=2, default=str)
        handle.write('\n')
    os.replace(path + '.tmp', path)


# --------------------------------------------------------------------------
# Analysis: pure functions over bytes, reads and open times
# --------------------------------------------------------------------------

def split_packets(data):
    """Split a byte stream like magnetometer_reader.py's read loop.

    Find 0xAA, need 218 bytes, require 0xBB at index 217, else drop one byte
    and search again. One extra check: a packet at the start of the data or
    right after skipped bytes must be followed by another 0xAA (or the end of
    the data). Otherwise it is a 0xAA among the float bytes of a partial packet
    that happens to have 0xBB 217 bytes later, inside the next real packet.

    Returns packets as (index of last byte, 54 floats), the resync count, the
    skipped byte ranges and the offset of the first packet.
    """
    packets, skipped = [], []
    resyncs, pos, n = 0, 0, len(data)

    def skip(start, end):
        if skipped and skipped[-1][1] == start:
            skipped[-1][1] = end
        else:
            skipped.append([start, end])

    while n - pos >= PACKET_SIZE:
        start = data.find(bytes([PACKET_HEADER]), pos)
        if start == -1:
            skip(pos, n)
            pos = n
            break
        if start > pos:
            skip(pos, start)
            pos = start
        if n - pos < PACKET_SIZE:
            break
        last = pos + PACKET_SIZE - 1
        after_gap = pos == 0 or (bool(skipped) and skipped[-1][1] == pos)
        misaligned = after_gap and last + 1 < n and data[last + 1] != PACKET_HEADER
        if data[last] != PACKET_TAIL or misaligned:
            resyncs += 1
            skip(pos, pos + 1)
            pos += 1
            continue
        packets.append((last, PACKET_STRUCT.unpack(bytes(data[pos + 1:last]))))
        pos += PACKET_SIZE
    first = packets[0][0] - (PACKET_SIZE - 1) if packets else n
    return {'packets': packets, 'resyncs': resyncs, 'skipped': skipped,
            'skipped_bytes': sum(e - s for s, e in skipped),
            'bytes_before_first_packet': first, 'leftover_bytes': n - pos}


def arrival_of(byte_indices, reads):
    """Map byte offsets to (t_ns, read index) of the read that delivered them."""
    ends, total = [], 0
    for _, nbytes in reads:
        total += nbytes
        ends.append(total)
    out = []
    for index in byte_indices:
        k = min(bisect.bisect_right(ends, index), len(reads) - 1)
        out.append((reads[k][0], k))
    return out


TEXT_PATTERN = re.compile(rb'[\t\x20-\x7e]{%d,}[\r\n]+' % TEXT_MIN_CHARS)


def find_text(data, skipped):
    """Printable lines inside skipped bytes: boot banners, versions, debug prints.

    A step usually starts in the middle of a packet, and float bytes often look
    like short text. So the partial packet (up to a 0xBB within one packet
    length of the skipped range's start) is dropped, and text must end in a
    line break. A real banner holds no 0xBB, so it is kept.
    """
    runs = []
    for start, end in skipped:
        tail = data.rfind(bytes([PACKET_TAIL]), start, min(end, start + PACKET_SIZE - 1))
        if tail != -1:
            start = tail + 1
        for match in TEXT_PATTERN.finditer(bytes(data[start:end])):
            runs.append((start + match.start(), match.group().decode('ascii')))
    return runs


def stream_pauses(times_ns):
    """(start ns, end ns) of each gap where the stream stopped (a reboot, a stall).

    A pause is longer than RESTART_DELAY_S and PAUSE_MEAN_GAPS mean gaps, so
    ordinary jitter of a slow stream is not one.
    """
    if len(times_ns) < 2:
        return []
    mean_gap = (times_ns[-1] - times_ns[0]) / (len(times_ns) - 1)
    limit = max(RESTART_DELAY_S * 1e9, PAUSE_MEAN_GAPS * mean_gap)
    return [(a, b) for a, b in zip(times_ns, times_ns[1:]) if b - a > limit]


def timing_stats(times_ns, read_indices):
    """Rate while streaming, pauses, packets per 1-s window, gap buckets and packets per read.

    The rate leaves out pauses: a board that reboots when the port opens can
    still deliver a few packets from before the reset, then nothing for a
    second or more, and counting that gap would understate its rate.
    """
    n = len(times_ns)
    out = {'packets': n}
    if n < 2 or times_ns[-1] == times_ns[0]:
        return out
    span_s = (times_ns[-1] - times_ns[0]) / 1e9
    pauses = stream_pauses(times_ns)
    paused_s = sum(b - a for a, b in pauses) / 1e9
    out['rate_hz'] = (n - 1) / span_s
    if pauses and n - 1 > len(pauses):
        out['rate_hz'] = (n - 1 - len(pauses)) / (span_s - paused_s)
    out['pauses_s'] = [(b - a) / 1e9 for a, b in pauses]
    windows = Counter(int((t - times_ns[0]) // 1e9) for t in times_ns)
    full = [windows.get(i, 0) for i in range(int(span_s))]      # complete windows only
    if full:
        out['per_second'] = {'min': min(full), 'median': statistics.median(full),
                             'max': max(full)}
    gaps = [(b - a) / 1e6 for a, b in zip(times_ns, times_ns[1:])]
    edges = (0.0,) + GAP_EDGES_MS + (float('inf'),)
    labels = ['<1 ms']
    labels += ['%d-%d ms' % pair for pair in zip(GAP_EDGES_MS, GAP_EDGES_MS[1:])]
    labels += ['>=100 ms']
    out['gap_fractions'] = {label: sum(lo <= g < hi for g in gaps) / len(gaps)
                            for label, lo, hi in zip(labels, edges, edges[1:])}
    out['max_gap_ms'] = max(gaps)
    out['median_packets_per_read'] = statistics.median(Counter(read_indices).values())
    return out


def field_stats(rows):
    """Per-channel mean/std/min/max, NaN/inf and exact-zero counts, per-sensor mean |B|."""
    out = {'channels': {}, 'nonfinite': 0, 'exact_zeros': 0, 'sensor_mean_abs_b': []}
    for c, name in enumerate(CHANNELS):
        values = [row[c] for row in rows]
        good = finite(values)
        if c < N_FIELD_CHANNELS:
            out['nonfinite'] += len(values) - len(good)
            out['exact_zeros'] += sum(v == 0.0 for v in good)
        m, s = mean_std(good)
        out['channels'][name] = None if m is None else {
            'mean': m, 'std': s, 'min': min(good), 'max': max(good)}
    for s in range(N_SENSORS):
        mags = [math.sqrt(sum(v * v for v in row[3 * s:3 * s + 3])) for row in rows]
        out['sensor_mean_abs_b'].append(mean_std(mags)[0])
    sensor_means = [v for v in out['sensor_mean_abs_b'] if v is not None]
    out['median_abs_b'] = statistics.median(sensor_means) if sensor_means else None
    stds = [ch['std'] for ch in list(out['channels'].values())[:N_FIELD_CHANNELS] if ch]
    out['median_std'] = statistics.median(stds) if stds else None
    flat = finite([v for row in rows for v in row[:N_FIELD_CHANNELS]])
    out['all_integer'] = bool(flat) and any(flat) and all(v == int(v) for v in flat)
    return out


def value_step(values):
    """Smallest step between the distinct readings of one channel, if every step is a multiple.

    The sensor sends whole counts; firmware that converts them multiplies by
    uT per count and may subtract a baseline, so the readings sit on a grid
    n * step - baseline. Exact zeros (sent before a baseline exists) are left out.
    """
    distinct = sorted({v for v in finite(values) if v != 0.0 and abs(v) < STEP_MAX_ABS})
    steps = [b - a for a, b in zip(distinct, distinct[1:])]
    if len(steps) < 3 or min(steps) <= 0:
        return None
    step = min(steps)
    if any(abs(d / step - round(d / step)) > 0.01 for d in steps):
        return None
    return step


def axis_steps(rows):
    """The value step shared by the X/Y channels and by the Z channels (None if unclear)."""
    out = {}
    for axis, offsets in (('xy', (0, 1)), ('z', (2,))):
        steps = [value_step([row[3 * s + o] for row in rows])
                 for s in range(N_SENSORS) for o in offsets]
        steps = [q for q in steps if q]
        out[axis] = None
        if steps:
            mid = statistics.median(steps)
            agree = [q for q in steps if abs(q / mid - 1) < STEP_TOL]
            if len(agree) >= STEP_MIN_CHANNELS:
                out[axis] = mid
    return out


def mlx_settings(step_xy, step_z):
    """(unit, value per uT, [(GAIN_SEL, RES), ...]) whose uT per count gives both steps."""
    if not step_xy or not step_z:
        return None, None, []
    for unit, per_ut in STEP_UNITS:
        found = []
        for gain, (xy, z) in enumerate(zip(MLX_UT_PER_COUNT_XY, MLX_UT_PER_COUNT_Z)):
            for res in range(4):
                if (abs(step_xy / (xy * 2 ** res * per_ut) - 1) < STEP_TOL
                        and abs(step_z / (z * 2 ** res * per_ut) - 1) < STEP_TOL):
                    found.append((gain, res))
        if found:
            return unit, per_ut, found
    return None, None, []


def settings_names(settings):
    return ', '.join('GAIN_SEL %d + RES %d' % pair for pair in settings)


def mlx_full_scale_ut(gain, res):
    """(XY, Z) full scale in uT of one MLX90393 setting (HALLCONF 0xC table)."""
    return (MLX_RES_COUNTS[res] * MLX_UT_PER_COUNT_XY[gain] * 2 ** res,
            MLX_RES_COUNTS[res] * MLX_UT_PER_COUNT_Z[gain] * 2 ** res)


def guess_units(far, steps=None):
    """Field unit: from the value steps if they match the MLX90393 table, else from Earth's field.

    `steps` is axis_steps() of the far step. The Earth-field guess uses the far
    step's median |B| (Earth's field is 25-65 uT).
    """
    b, noise = far.get('median_abs_b'), far.get('median_std') or 0.0
    out = {'median_abs_b': b, 'noise': noise, 'unit': None, 'to_ut': None, 'baseline': None,
           'confidence': 'unclear', 'text': 'no far-step packets', 'settings': []}
    if b is None:
        return out
    evidence = 'far-step median |B| = %s, noise (median std) = %s' % (fmt(b), fmt(noise))
    baseline = b < BASELINE_NOISE_RATIO * noise or b == 0.0
    steps = steps or {}
    unit, per_ut, settings = mlx_settings(steps.get('xy'), steps.get('z'))
    if unit:
        b_ut = b / per_ut
        baseline = b_ut < UNIT_WINDOWS[3][1]    # below the lowest Earth-field reading
        text = ('%s: X/Y readings move in steps of %s and Z in steps of %s, the MLX90393 '
                'uT per count at %s' % (unit, fmt(steps['xy']), fmt(steps['z']),
                                        settings_names(settings)))
        text += ('; far from the magnet |B| is only %s uT (Earth: 25-65 uT), so a baseline is '
                 'subtracted' % fmt(b_ut, 3) if baseline else
                 '; far from the magnet |B| is %s uT, so the Earth field is in the data'
                 % fmt(b_ut, 3))
        out.update(unit=unit, to_ut=1.0 / per_ut, baseline=baseline, confidence='measured',
                   settings=settings, steps=steps, text='%s (%s)' % (text, evidence))
        return out
    if far.get('all_integer'):
        text = 'raw sensor counts: all values are integers'
        if baseline:
            text += ', and a baseline is probably subtracted'
        out.update(unit='counts', baseline=baseline, confidence='likely',
                   text='%s (%s; Earth would be about 167-433 counts at GAIN_SEL 7, RES 0)'
                        % (text, evidence))
    elif baseline:
        out.update(baseline=True, confidence='likely',
                   text='a baseline is probably subtracted, so the units cannot be read from '
                        'the Earth field (%s)' % evidence)
    else:
        out['text'] = "no unit puts the far |B| near Earth's 25-65 uT (%s)" % evidence
        for unit, low, high, factor in UNIT_WINDOWS:
            if low <= b <= high:
                out.update(unit=unit, to_ut=factor, baseline=False, confidence='likely',
                           text="%s: far from the magnet |B| is %s uT, inside Earth's 25-65 uT "
                                "(%s)" % (unit, fmt(b * factor, 3), evidence))
                break
        if out['unit'] == 'mG':     # raw counts at the Adafruit default land here too
            out['confidence'] = 'unclear'
            out['text'] += '; or raw counts at GAIN_SEL 7, RES 0 (averaged, so not whole numbers)'
    return out


def longest_run(values, target):
    best = run = 0
    for v in values:
        run = run + 1 if v == target else 0
        best = max(best, run)
    return best


def flat_top_run(cols, c, peak):
    """Longest run of samples within FLAT_TOL of `peak` while the sensor's other axes moved.

    A real field component can sit at its maximum for a moment, but not while
    another axis of the same sensor changes by a fifth of that value: then the
    channel has stopped rising (a limit that is not an exact repeated number).
    """
    first = c - c % 3
    others = [cols[o] for o in range(first, first + 3) if o != c]
    best, start = 0, None
    for i, v in enumerate(list(cols[c]) + [None]):
        near = v is not None and math.isfinite(v) and abs(v) >= (1 - FLAT_TOL) * peak
        if near and start is None:
            start = i
        elif not near and start is not None:
            if i - start >= FLAT_MIN_RUN:
                change = max(max(part) - min(part)
                             for part in (finite(o[start:i]) or [0.0] for o in others))
                if change >= FLAT_OTHER_CHANGE * peak:
                    best = max(best, i - start)
            start = None
    return best


def detect_saturation(magnet_rows, far):
    """Look for pinned extremes (clipping), flat tops, sign flips and shared limits.

    A channel counts as clipped only if its largest or smallest value repeats
    exactly in a row, is large (vs every channel and vs the far-step noise),
    and its sensor moved a lot in that step (on any of its three axes). So a
    still magnet with quantised readings is not flagged, but a channel pinned
    for the whole step is. A sign flip is a jump between two large values of
    opposite sign: a fast pass over a sensor, or an int16 wrap. A flat top is
    a channel that stays near its largest value while its sensor keeps
    changing (see flat_top_run); the X/Y channels of the real board do this.
    """
    out = {'global_max_abs': None, 'max_abs_xy': None, 'max_abs_z': None,
           'clipped': [], 'flat_tops': [], 'wraps': [], 'shared_limits': []}
    series = {}
    for step, rows in magnet_rows.items():
        if rows:
            series[step] = [[row[c] for row in rows] for c in range(N_FIELD_CHANNELS)]
    if not series:
        return out
    every = []
    for c in range(N_FIELD_CHANNELS):
        every.append(finite([v for cols in series.values() for v in cols[c]]) or [0.0])
    chan_max = [max(abs(v) for v in values) for values in every]
    chan_span = [max(values) - min(values) for values in every]
    top = max(chan_max)
    out.update(global_max_abs=top, max_abs_z=max(chan_max[2::3]),
               max_abs_xy=max(m for c, m in enumerate(chan_max) if c % 3 != 2),
               channel_max_abs=dict(zip(CHANNELS, chan_max)))
    far_channels = far.get('channels') or {}
    floor = max(far.get('median_std') or 0.0, 1e-9)
    for step, cols in series.items():
        spans = []
        for values in cols:
            good = finite(values)
            spans.append(max(good) - min(good) if good else 0.0)
        for c, values in enumerate(cols):
            good = finite(values)
            if not good:
                continue
            noise = max((far_channels.get(CHANNELS[c]) or {}).get('std') or 0.0, floor)
            first_axis = c - c % 3
            sensor_span = max(spans[first_axis:first_axis + 3])
            moved = sensor_span >= CLIP_MIN_RANGE * max(abs(max(good)), abs(min(good)))
            for extreme in sorted({max(good), min(good)}):
                run = longest_run(values, extreme)
                if (run >= CLIP_MIN_RUN and moved and abs(extreme) >= CLIP_MIN_FRACTION * top
                        and abs(extreme) >= CLIP_MIN_NOISE * noise):
                    out['clipped'].append({'step': step, 'channel': CHANNELS[c],
                                           'value': extreme, 'run': run})
            axis_top = out['max_abs_z'] if c % 3 == 2 else out['max_abs_xy']
            flat = flat_top_run(cols, c, chan_max[c])
            pinned = any(x['step'] == step and x['channel'] == CHANNELS[c] for x in out['clipped'])
            rose = spans[c] >= CLIP_MIN_FRACTION * chan_max[c]    # not a constant offset
            if (flat and rose and not pinned and chan_max[c] >= CLIP_MIN_FRACTION * axis_top
                    and chan_max[c] >= CLIP_MIN_NOISE * noise):
                out['flat_tops'].append({'step': step, 'channel': CHANNELS[c],
                                         'value': chan_max[c], 'run': flat})
            for i, (a, b) in enumerate(zip(values, values[1:])):
                if (a * b < 0 and abs(a - b) > WRAP_MIN_FRACTION * chan_span[c]
                        and min(abs(a), abs(b)) >= WRAP_MIN_FRACTION * chan_max[c]):
                    out['wraps'].append({'step': step, 'channel': CHANNELS[c], 'sample': i + 1,
                                         'from': a, 'to': b})
    # Shared limit: several channels stop at the exact same magnet-sized magnitude.
    magnet_sized = max(CLIP_MIN_FRACTION * top, CLIP_MIN_NOISE * floor,
                       2.0 * (far.get('median_abs_b') or 0.0))
    groups = Counter(m for m in chan_max if m > 0 and m >= magnet_sized)
    for magnitude, count in groups.items():
        if count >= 2:
            names = [CHANNELS[c] for c in range(N_FIELD_CHANNELS) if chan_max[c] == magnitude]
            out['shared_limits'].append({'magnitude': magnitude, 'channels': names})
    return out


def full_scale_note(sat, units):
    """Compare the largest readings with MLX90393 full-scale values, without overclaiming."""
    xy, z = sat.get('max_abs_xy'), sat.get('max_abs_z')
    if xy is None:
        return 'no magnet-step data'
    if units.get('unit') == 'counts':
        return ('largest |value| %s counts (int16 limit at RES 0/1 is 32768; RES 2 spans '
                '+/-22000, RES 3 +/-11000)' % fmt(max(xy, z)))
    if not units.get('to_ut'):
        return 'largest |value| %s (units unknown, so no full-scale comparison)' % fmt(max(xy, z))
    xy_ut, z_ut = xy * units['to_ut'], z * units['to_ut']
    text = 'largest |Bxy| = %s uT, |Bz| = %s uT' % (fmt(xy_ut, 3), fmt(z_ut, 3))
    if units.get('settings'):
        return text + settings_full_scale(xy_ut, z_ut, units['settings'])
    at_limit = [name for name, fs_xy, fs_z in FULL_SCALE_UT
                if abs(xy_ut - fs_xy) <= FULL_SCALE_TOL * fs_xy
                or abs(z_ut - fs_z) <= FULL_SCALE_TOL * fs_z]
    if at_limit:
        return text + ('; matches the full scale of %s: probably clipped or wrapped'
                       % '; '.join(at_limit))
    over = [name for name, fs_xy, fs_z in FULL_SCALE_UT if xy_ut > fs_xy or z_ut > fs_z]
    if not over:
        return text + '; below the smallest MLX90393 full scale (4915/7930 uT)'
    text += ('; above the full scale of %s with the HALLCONF 0xC table (another table or '
             'HALLCONF allows up to ~30 %% more)' % '; '.join(over))
    if max(xy_ut, z_ut) > HALL_SATURATION_UT:
        text += ('; above the 50 mT Hall-plate saturation onset, so those readings are not '
                 'trustworthy')
    return text


def settings_full_scale(xy_ut, z_ut, settings):
    """How much of each matching setting's nominal range the largest readings use.

    Readings above a nominal range do not rule a setting out: RES 2 and 3 are
    specified as +/-22000 and +/-11000 counts, but the output can go further.
    """
    used, over, at_limit = [], [], []
    for gain, res in settings:
        fs_xy, fs_z = mlx_full_scale_ut(gain, res)
        name = 'GAIN_SEL %d + RES %d (%s / %s uT)' % (gain, res, fmt(fs_xy), fmt(fs_z))
        if (abs(xy_ut - fs_xy) <= FULL_SCALE_TOL * fs_xy
                or abs(z_ut - fs_z) <= FULL_SCALE_TOL * fs_z):
            at_limit.append(name)
        else:
            share = 'GAIN_SEL %d + RES %d: %.0f %% / %.0f %% of %s / %s uT' % (
                gain, res, 100 * xy_ut / fs_xy, 100 * z_ut / fs_z, fmt(fs_xy), fmt(fs_z))
            (over if xy_ut > fs_xy or z_ut > fs_z else used).append(share)
    text = ''
    if at_limit:
        text += '; matches the full scale of %s: probably clipped or wrapped' % '; '.join(at_limit)
    if used:
        text += '; range used (XY / Z): ' + '; '.join(used)
    if over:
        text += ('; above the nominal range of ' + '; '.join(over)
                 + ' (so with that setting these readings are at or past its limit)')
    if max(xy_ut, z_ut) > HALL_SATURATION_UT:
        text += ('; above the 50 mT Hall-plate saturation onset, so those readings are not '
                 'trustworthy')
    return text


def pose_stats(rows):
    """Pose ranges and |m| = sqrt(mx^2+my^2+mz^2) mean and relative spread."""
    out = {}
    for axis in ('x', 'y', 'z'):
        good = finite([row[POSE[axis]] for row in rows])
        out[axis] = None
        if good:
            out[axis] = {'min': min(good), 'max': max(good), 'median': statistics.median(good)}
    moments = [math.sqrt(row[POSE['mx']] ** 2 + row[POSE['my']] ** 2 + row[POSE['mz']] ** 2)
               for row in rows]
    m, s = mean_std(moments)
    out['m_mean'], out['m_spread'] = m, (s / m if m else None)
    return out


def guess_pose_units(sweep_rows):
    """Metres if the sweep's 95th-percentile |x|,|y| is under 0.2; mm if it is tens of units.

    The pose must also follow the stylus: a sweep range under 3 cm in x or y
    means it does not (a frozen or failed fit), so no unit is claimed.
    """
    xs = finite([row[POSE['x']] for row in sweep_rows])
    ys = finite([row[POSE['y']] for row in sweep_rows])
    xy = sorted(abs(v) for v in xs + ys)
    if not xy:
        return {'unit': None, 'confidence': 'unclear', 'text': 'no sweep packets'}
    p95 = xy[int(0.95 * (len(xy) - 1))]
    span = min(max(v) - min(v) if v else 0.0 for v in (xs, ys))
    outside = sum(v > POSE_LIMIT_M for v in xy) / len(xy)
    evidence = ("sweep 95th-percentile |x|,|y| = %s, smaller of the x and y ranges = %s, "
                "%.0f %% of values beyond the reader's 0.05 m window"
                % (fmt(p95), fmt(span), 100 * outside))
    if p95 == 0.0:
        unit, conf, text = None, 'measured', 'pose is all zero: the board does not compute it'
    elif p95 <= 0.2 or 5.0 <= p95 <= 500.0:
        unit = 'm' if p95 <= 0.2 else 'mm'
        span_m = span if unit == 'm' else span / 1000.0
        if span_m < POSE_MIN_SWEEP_M:
            unit, conf, text = None, 'unclear', 'pose does not follow the stylus'
        else:
            conf, text = 'likely', 'metres' if unit == 'm' else 'millimetres'
    else:
        unit, conf, text = None, 'unclear', 'neither metres nor mm'
    return {'unit': unit, 'confidence': conf, 'p95': p95, 'span': span,
            'text': '%s (%s)' % (text, evidence)}


def interpret_moment(magnet_rows):
    """Unit direction vector, constant or fitted strength, from |m| over the magnet steps."""
    stats = pose_stats([row for rows in magnet_rows.values() for row in rows])
    m, spread = stats['m_mean'], stats['m_spread']
    if m is not None and m < ZERO_MOMENT:
        return {'kind': 'zero', 'confidence': 'measured', 'm_mean': m,
                'text': 'moment is all zero: the board does not compute it'}
    if m is None or spread is None:
        return {'kind': None, 'confidence': 'unclear', 'text': 'no magnet-step packets'}
    if abs(m - 1.0) < UNIT_MOMENT_TOL and spread < STEADY_SPREAD:
        kind, conf, text = 'unit', 'measured', ('a unit direction vector (the dipole axis), '
                                                'as colmag/control_mapping.py assumes')
    elif spread < STEADY_SPREAD:
        kind, conf, text = 'fixed', 'likely', ('constant strength (fixed in firmware, or a '
                                               'well-fitted permanent magnet); only the '
                                               'direction is used')
    elif spread > FITTED_SPREAD:
        kind, conf, text = 'fitted', 'likely', ('a fitted moment whose strength changes (or a '
                                                'noisy fit); only its direction is used')
    else:
        kind, conf, text = 'unclear', 'unclear', ('nearly constant length, but not clearly a '
                                                  'unit vector')
    evidence = '|m| mean %s, relative spread %.1f %%' % (fmt(m), 100 * spread)
    return {'kind': kind, 'confidence': conf, 'm_mean': m, 'm_spread': spread,
            'text': '%s (%s)' % (text, evidence)}


def shift_in_noise(far_rows, other_rows):
    """Per-channel mean shift from the far step, and its size in far-step noise units.

    The noise is floored so a perfectly quiet channel cannot divide by ~0.
    """
    far_mean, other_mean = column_means(far_rows), column_means(other_rows)
    stds = [mean_std([row[c] for row in far_rows])[1] or 0.0 for c in range(N_FIELD_CHANNELS)]
    floor = max(statistics.median(stds),
                1e-3 * statistics.median([abs(v or 0.0) for v in far_mean]), 1e-9)
    shift = [(o or 0.0) - (f or 0.0) for o, f in zip(other_mean, far_mean)]
    sigmas = [abs(s) / max(sd, floor) for s, sd in zip(shift, stds)]
    return shift, sigmas


def moved_fraction(sigmas):
    """Fraction of channels that shifted by more than OPEN_SHIFT_SIGMA noise units."""
    return sum(v > OPEN_SHIFT_SIGMA for v in sigmas) / float(len(sigmas))


def pose_range(rows):
    """Largest range of Pose_x or Pose_y in metres (0 when the pose is frozen)."""
    ranges = [0.0]
    for axis in ('x', 'y'):
        good = finite([row[POSE[axis]] for row in rows])
        if good:
            ranges.append(max(good) - min(good))
    return max(ranges)


def port_open_effect(far_rows, still_rows, reopen_far_rows, reopen_magnet_rows=None):
    """Did reopening the port (with the stylus on the board) move the far-field readings?

    If the board captures its baseline at open, the stylus field at that
    moment is baked in, so after the reopen the far readings shift by minus
    that field. The still step had the stylus at the same place, so it gives
    the expected shift: -(still - far). reopen_magnet cannot give it (with a
    re-captured baseline it reads about zero), but it must differ from far
    when nothing is re-captured: if it does not, the stylus was not seen at the
    reopen and nothing can be concluded.
    """
    if not far_rows or not reopen_far_rows:
        return {'result': None, 'confidence': 'unclear',
                'text': 'no reopen data (run all steps to answer this)'}
    shift, sigmas = shift_in_noise(far_rows, reopen_far_rows)
    moved = moved_fraction(sigmas)
    out = {'shift_sigma_median': statistics.median(sigmas), 'shift_sigma_max': max(sigmas),
           'moved_fraction': moved}
    evidence = 'reopen_far vs far: median %.1f, max %.1f noise units, %.0f %% of channels moved' % (
        out['shift_sigma_median'], out['shift_sigma_max'], 100 * moved)
    magnet_moved = None
    if reopen_magnet_rows:
        magnet_moved = moved_fraction(shift_in_noise(far_rows, reopen_magnet_rows)[1])
        out['reopen_magnet_moved_fraction'] = magnet_moved
    if magnet_moved is not None and magnet_moved >= OPEN_SHIFT_FRACTION:
        # A baseline taken at the reopen would have absorbed the stylus field, so
        # reopen_magnet would read like far. It did not, so nothing was re-captured.
        text = ('no baseline is re-captured at port open: right after the reopen the stylus '
                'field was still visible (%.0f %% of channels differ from far)'
                % (100 * magnet_moved))
        if moved >= OPEN_SHIFT_FRACTION:
            if pose_range(far_rows) < POSE_FROZEN_M <= pose_range(reopen_far_rows):
                text += ('; in reopen_far the firmware still fitted a magnet (its pose moved '
                         'by %s m; with no magnet in the far step it stayed frozen): either the '
                         'stylus was not far enough away, or the sensors kept an offset after '
                         'the strong field (repeat with the stylus in another room to tell)'
                         % fmt(pose_range(reopen_far_rows), 2))
            else:
                text += '; reopen_far still differs from far: drift, or the stylus was not fully away'
        out.update(result='none', confidence='measured', text='%s (%s)' % (text, evidence))
        return out
    if moved < OPEN_SHIFT_FRACTION:
        if magnet_moved is not None:
            out.update(result=None, confidence='unclear',
                       text='the stylus was not seen during reopen_magnet: repeat step 5 '
                            'with the stylus on the board (%s)' % evidence)
            return out
        out.update(result='none', confidence='measured',
                   text='no baseline is re-captured at port open (%s)' % evidence)
        return out
    expected = [0.0] * N_FIELD_CHANNELS
    if still_rows:
        pairs = zip(column_means(still_rows), column_means(far_rows))
        expected = [-((s or 0.0) - (f or 0.0)) for s, f in pairs]
    dot = sum(a * b for a, b in zip(shift, expected))
    norm = math.sqrt(sum(a * a for a in shift) * sum(b * b for b in expected))
    out['correlation'] = dot / norm if norm else 0.0
    out['ratio'] = dot / sum(b * b for b in expected) if any(expected) else 0.0
    evidence += ', correlation with -(still - far) %.2f, size ratio %.2f' % (
        out['correlation'], out['ratio'])
    if out['correlation'] > 0.8 and 0.5 <= out['ratio'] <= 1.5:
        out.update(result='baseline', confidence='measured',
                   text='the board captures its baseline when the port opens, so always open '
                        'the port with the stylus far away (%s)' % evidence)
    else:
        out.update(result='other', confidence='unclear',
                   text='readings changed after the reopen, but not like a re-captured baseline; '
                        'maybe drift, or the stylus was not fully away (%s)' % evidence)
    return out


def restart_evidence(steps):
    """Firmware text right after opening, or a slow first packet, suggests a reboot on open.

    If only the first open shows it and the reopen does not, the firmware ran
    its start-up once (typically it waits for the first connection after
    power-up) and opening the port again does not restart it.
    """
    signs, delays, opens = [], [], []
    for name, step in steps.items():
        if not step.get('opened_here'):
            continue
        step_signs = []
        delay = step['timing'].get('open_to_first_packet_s')
        if delay is not None:
            delays.append('%s %.3f s' % (name, delay))
            if delay > RESTART_DELAY_S:
                step_signs.append('first packet %.2f s after opening in %s' % (delay, name))
        pause = step['timing'].get('pause_after_open_s')
        if pause is not None:
            step_signs.append('stream stopped for %.2f s right after opening in %s'
                              % (pause, name))
        early = [t for t in step['text']
                 if t['after_open_s'] is not None and t['after_open_s'] < RESTART_TEXT_S]
        if early:
            step_signs.append('firmware text right after opening in %s' % name)
        signs += step_signs
        if delay is not None:
            opens.append((name, delay, step_signs))
    if not delays:
        return {'restart': None, 'confidence': 'unclear', 'text': 'no packets after an open'}
    if len(opens) > 1 and opens[0][2] and not any(later[2] for later in opens[1:]):
        return {'restart': False, 'confidence': 'likely',
                'text': 'start-up only at the first open (%s), none at the reopen (first packet '
                        '%s s after opening, no text): opening the port again does not restart '
                        'the board; the start-up probably runs once after power-up'
                        % ('; '.join(opens[0][2]), fmt(opens[1][1], 2))}
    if signs:
        return {'restart': True, 'confidence': 'likely',
                'text': 'probable restart on open: ' + '; '.join(signs)}
    return {'restart': False, 'confidence': 'likely',
            'text': 'no sign of a restart on open (first packet after: %s; no text)'
                    % ', '.join(delays)}


def board_answer(info):
    """Board family from the USB vendor id of the chosen port."""
    if not info or info.get('vid') is None:
        text = 'no USB vendor id for this port'
        names = (info or {}).get('by_id') or []
        if names:
            text += '; its /dev/serial/by-id name is %s' % os.path.basename(names[0])
        text += '; to see the USB identity, on the host run: ls -l /dev/serial/by-id/ or lsusb'
        return {'confidence': 'unclear', 'text': text}
    vid, pid = info['vid'], info.get('pid') or 0
    family = USB_VENDORS.get(vid, 'unknown vendor')
    name = info.get('description') or info.get('device')
    text = '%s, VID:PID %04X:%04X -> %s' % (name, vid, pid, family)
    if vid in BRIDGE_VIDS:
        text += ('; a USB-UART bridge usually toggles DTR/RTS on open and may reset the '
                 'microcontroller')
    return {'confidence': 'likely' if vid in USB_VENDORS else 'unclear', 'text': text,
            'family': family}


def analyze_step(data, reads, open_ns, opened_here):
    """All numbers for one step. Returns (JSON-able dict, list of 54-float rows)."""
    split = split_packets(data)
    rows = [floats for _, floats in split['packets']]
    arrivals = arrival_of([last for last, _ in split['packets']], reads) if reads else []
    timing = timing_stats([t for t, _ in arrivals], [k for _, k in arrivals])
    since_open = opened_here and open_ns is not None and reads
    if since_open:
        timing['open_to_first_byte_s'] = (reads[0][0] - open_ns) / 1e9
        if arrivals:
            timing['open_to_first_packet_s'] = (arrivals[0][0] - open_ns) / 1e9
            early = [b - a for a, b in stream_pauses([t for t, _ in arrivals])
                     if a - open_ns < RESTART_TEXT_S * 1e9]
            if early:
                timing['pause_after_open_s'] = max(early) / 1e9
    texts = []
    for offset, text in find_text(data, split['skipped']):
        after = (arrival_of([offset], reads)[0][0] - open_ns) / 1e9 if since_open else None
        texts.append({'offset': offset, 'text': text, 'after_open_s': after})
    leading_zeros = 0
    for row in rows:
        if any(row[:N_FIELD_CHANNELS]):
            break
        leading_zeros += 1
    result = {'bytes': len(data), 'reads': len(reads), 'opened_here': opened_here,
              'timing': timing, 'text': texts, 'fields': field_stats(rows),
              'pose': pose_stats(rows) if rows else None, 'leading_zero_packets': leading_zeros}
    for key in ('resyncs', 'skipped_bytes', 'bytes_before_first_packet', 'leftover_bytes'):
        result[key] = split[key]
    return result, rows


def far_stream_ok(step):
    """Enough packets and little garbage in the far step (else wrong port, baud or a 2nd reader).

    Random bytes hold a stray 'packet' about once per 65 kB, so a few packets
    prove nothing. The partial packet at the start (under 218 bytes) is not garbage.
    """
    garbage = step['skipped_bytes'] - min(step['bytes_before_first_packet'], PACKET_SIZE - 1)
    return (step['timing']['packets'] >= MIN_PACKETS
            and garbage <= MAX_GARBAGE_FRACTION * step['bytes'])


# --------------------------------------------------------------------------
# Answers and report
# --------------------------------------------------------------------------

def analyze_folder(folder):
    """Analyse a saved probe folder (the live run and --replay both use this)."""
    meta = read_json(os.path.join(folder, 'meta.json'))
    steps, rows = {}, {}
    for step in meta.get('steps', []):
        name = step['name']
        with open(os.path.join(folder, name + '.bin'), 'rb') as handle:
            data = handle.read()
        with open(os.path.join(folder, name + '_reads.csv'), newline='') as handle:
            reads = [(int(r['t_ns']), int(r['nbytes'])) for r in csv.DictReader(handle)]
        opened_here = step.get('opened_here', False)
        steps[name], rows[name] = analyze_step(data, reads, step.get('open_ns'), opened_here)
        steps[name]['duration_s'] = (step['end_ns'] - step['start_ns']) / 1e9
        steps[name]['stopped'] = step.get('stopped')
    far = steps['far']['fields'] if 'far' in steps else {}
    magnet_rows = {name: rows[name] for name in MAGNET_STEPS if name in rows}
    units = guess_units(far, axis_steps(rows['far']) if 'far' in rows else None)
    saturation = detect_saturation(magnet_rows, far)
    saturation['full_scale'] = full_scale_note(saturation, units)
    port_open = port_open_effect(rows.get('far'), rows.get('still'), rows.get('reopen_far'),
                                 rows.get('reopen_magnet'))
    results = {
        'tool_version': TOOL_VERSION, 'folder': os.path.abspath(folder), 'meta': meta,
        'steps': steps, 'board': board_answer(meta.get('port_info')), 'units': units,
        'saturation': saturation, 'pose_units': guess_pose_units(rows.get('sweep', [])),
        'moment': interpret_moment(magnet_rows), 'port_open': port_open,
        'restart': restart_evidence(steps),
    }
    results['answers'] = build_answers(results)
    return results


def rate_answers(r):
    """Sample rate and whether host timestamps are usable, from the far step."""
    step = r['steps'].get('far')
    if step is None:
        return [('Sample rate', 'not measured: no far step was recorded', 'unclear')]
    far = step['timing']
    if not far_stream_ok(step) or not far.get('rate_hz'):
        return [('Sample rate', '%d packet(s) and %d skipped bytes out of %d in the far step: %s'
                 % (far['packets'], step['skipped_bytes'], step['bytes'], WRONG_PORT_HINT),
                 'unclear')]
    windows = far.get('per_second')
    per_window = ''
    if windows:
        per_window = '; per 1-s window min/median/max %s/%s/%s' % (
            windows['min'], windows['median'], windows['max'])
    vid = (r['meta'].get('port_info') or {}).get('vid')
    link = ''
    if vid in BRIDGE_VIDS:
        cap = r['meta'].get('baudrate', 921600) / 10.0 / PACKET_SIZE
        link = '; the UART link carries at most ~%d/s' % cap
    elif vid is not None:
        link = '; native USB: the baud rate does not limit the rate'
    pauses = far.get('pauses_s') or []
    paused = ''
    if pauses:
        paused = ('; not counting %d pause(s) of %s s in total where no packets came'
                  % (len(pauses), fmt(sum(pauses), 3)))
    rate = ('Sample rate', '%.1f packets/s in the far step (%d packets%s%s%s)'
            % (far['rate_hz'], far['packets'], paused, per_window, link), 'measured')
    batched = far['gap_fractions']['<1 ms']
    usable = batched <= BATCHED_FRACTION
    timing = ('Host timing usable for speed',
              '%s: %.0f %% of packets arrive <1 ms after the previous one, i.e. in bursts that '
              'USB hands over together (median %s packets per read), max gap %.1f ms%s'
              % ('yes' if usable else 'no', 100 * batched, fmt(far['median_packets_per_read'], 3),
                 far['max_gap_ms'], '' if usable else '; use the packet count / mean rate instead'),
              'measured')
    return [rate, timing]


def saturation_answer(sat):
    """(text, confidence): pinned values say 'yes', flat tops 'probably'; sign flips are unclear."""
    clipped = ['%s pinned at %s for %d samples in %s' % (c['channel'], fmt(c['value']), c['run'],
                                                        c['step'])
               for c in sat['clipped'][:4]]
    flat = ['%s stays within %d %% of %s for %d samples in %s' % (
                f['channel'], round(100 * FLAT_TOL), fmt(f['value']), f['run'], f['step'])
            for f in sat['flat_tops'][:4]]
    parts = ['yes: ' + '; '.join(clipped)] if clipped else []
    if flat:
        parts.append(('' if clipped else 'probably: ') + '; '.join(flat) + (
            ' while another axis of the same sensor changed by over %d %% of that value, so '
            'the channel stopped rising there' % round(100 * FLAT_OTHER_CHANGE)))
    if parts:
        text = '; '.join(parts)
    elif sat['wraps']:
        text = 'no clipped channels found'
    else:
        text = 'no clipped or wrapped channels found'
    if sat['wraps']:
        flips = ['%s jumps %s -> %s in %s' % (w['channel'], fmt(w['from']), fmt(w['to']), w['step'])
                 for w in sat['wraps'][:4]]
        text += ('; sign flip between samples (a fast pass over a sensor, or an int16 wrap): '
                 + '; '.join(flips))
    for group in sat['shared_limits'][:2]:
        text += '; %d channels share the same largest |value| %s (a common full-scale limit?)' % (
            len(group['channels']), fmt(group['magnitude']))
    text += '; ' + sat['full_scale']
    if clipped or flat or 'probably clipped' in sat['full_scale']:
        return text, 'likely'
    return text, 'unclear' if sat['wraps'] else 'measured'


def build_answers(r):
    """One (question, answer with evidence, confidence) per question."""
    answers = []
    if r['meta'].get('error'):
        answers.append(('Recording failed', r['meta']['error'], 'measured'))
    answers.append(('Board', r['board']['text'], r['board']['confidence']))
    answers += rate_answers(r)

    units, port_open = r['units'], r['port_open']
    answers.append(('Units', units['text'], units['confidence']))
    if port_open.get('result') == 'baseline':
        answers.append(('Baseline subtracted', 'yes, captured when the port opens (see next line)',
                        'measured'))
    elif units.get('baseline') is True:
        far_step = r['steps'].get('far') or {}
        zeros = far_step.get('leading_zero_packets') or 0
        if zeros and far_step.get('opened_here'):
            answers.append(('Baseline subtracted', 'yes, taken at start-up: far |B| is at the '
                            'noise level, and the first %d packet(s) after opening the port were '
                            'exactly 0 (sent before the baseline existed)' % zeros, 'likely'))
        else:
            answers.append(('Baseline subtracted', 'probably: far |B| is at the noise level',
                            'likely'))
    elif units.get('baseline') is False:
        answers.append(('Baseline subtracted', 'no: far |B| is well above the noise, so the Earth '
                        'field is in the data', 'likely'))
    else:
        answers.append(('Baseline subtracted', 'cannot tell from these numbers', 'unclear'))
    open_conf = port_open['confidence'] if port_open.get('result') else r['restart']['confidence']
    answers.append(('Port open resets or recalibrates',
                    '%s; %s' % (r['restart']['text'], port_open['text']), open_conf))

    magnet_recorded = any(name in r['steps'] for name in MAGNET_STEPS)
    if not magnet_recorded:
        answers.append(('Saturation', NOT_MEASURED, 'unclear'))
    elif r['saturation']['global_max_abs'] is None:
        answers.append(('Saturation', 'no magnet-step packets', 'unclear'))
    else:
        answers.append(('Saturation',) + saturation_answer(r['saturation']))

    pose_text = r['pose_units']['text']
    still = (r['steps'].get('still') or {}).get('pose') or {}
    if still.get('z') and r['pose_units'].get('unit') == 'm':
        x, y, z = (still[axis]['median'] for axis in ('x', 'y', 'z'))
        pose_text += ('; stylus on the cover at the centre (still step): Pose x, y = %s, %s m '
                      '(the origin if both are near 0), Pose_z = %s m (control_mapping.py '
                      'subtracts %s m -> %s m)' % (fmt(x, 3), fmt(y, 3), fmt(z, 3),
                                                   MAGNET_HEIGHT_SENSOR_BIAS_M,
                                                   fmt(calibrated_magnet_height(z), 3)))
    if 'sweep' in r['steps']:
        answers.append(('Pose units', pose_text, r['pose_units']['confidence']))
    else:
        answers.append(('Pose units', NOT_MEASURED, 'unclear'))
    if magnet_recorded:
        answers.append(('What mx/my/mz mean', r['moment']['text'], r['moment']['confidence']))
    else:
        answers.append(('What mx/my/mz mean', NOT_MEASURED, 'unclear'))
    return answers


def firmware_questions(r):
    """Only what the probe could not answer (not what a skipped step would answer)."""
    settings = r['units'].get('settings')
    questions = [('Which of %s the firmware uses, and its OSR and DIG_FILT settings.'
                  % settings_names(settings)) if settings else
                 'Which MLX90393 settings (GAIN_SEL, RES, OSR, DIG_FILT, HALLCONF) and which '
                 'sensitivity table the firmware uses.',
                 'Which sensor number sits where on the board (the packets only say Sensor1..16).']
    if r['units']['confidence'] == 'unclear':
        questions.append('The exact field unit and scale factor (the probe can only guess: %s).'
                         % (r['units']['unit'] or 'unclear'))
    if not r['saturation']['clipped']:
        questions.append('What the firmware sends when a sensor overflows '
                         '(clip, wrap, or an error value).')
    if 'reopen_far' in r['steps'] and r['port_open'].get('result') in (None, 'other'):
        questions.append('Whether the board takes a baseline or restarts when the port opens.')
    pose_unclear = 'unclear' in (r['moment']['confidence'], r['pose_units']['confidence'])
    if 'sweep' in r['steps'] and pose_unclear:
        questions.append('How Pose_x..Pose_mz are computed and in which units.')
    return questions


def write_report(results, folder):
    """Write report.md and report.json; return the Answers as plain text for the terminal."""
    answers = results['answers']
    markdown = '\n'.join('- **%s** (%s): %s' % (q, conf, text) for q, text, conf in answers)
    plain = '\n'.join('- %s (%s): %s' % (q, conf, text) for q, text, conf in answers)
    meta = results['meta']
    lines = ['# Board probe %s' % os.path.basename(results['folder']), '',
             'Recorded %s on %s at %s baud (probe version %s).' % (
                 meta.get('started_local'), meta.get('port'), meta.get('baudrate'),
                 results['tool_version']),
             '', '## Answers', '', markdown, '', '## Steps', '',
             '| step | s | packets | rate /s | <1 ms gaps | resyncs | skipped B | NaN/inf | zeros '
             '| median abs B | noise |',
             '|---|---|---|---|---|---|---|---|---|---|---|']
    for name, s in results['steps'].items():
        t, f = s['timing'], s['fields']
        batched = 'n/a'
        if 'gap_fractions' in t:
            batched = '%.0f %%' % (100 * t['gap_fractions']['<1 ms'])
        label = name + (' (stopped: %s)' % s['stopped'] if s.get('stopped') else '')
        lines.append('| %s | %.1f | %d | %s | %s | %d | %d | %d | %d | %s | %s |' % (
            label, s['duration_s'], t['packets'], fmt(t.get('rate_hz')), batched, s['resyncs'],
            s['skipped_bytes'], f['nonfinite'], f['exact_zeros'], fmt(f['median_abs_b']),
            fmt(f['median_std'])))
    lines += ['', '## Pose per step', '',
              '| step | x min..max | y min..max | z median | abs m mean | abs m spread |',
              '|---|---|---|---|---|---|']
    for name, s in results['steps'].items():
        p = s['pose']
        if p and p['x'] and p['m_spread'] is not None:
            lines.append('| %s | %s..%s | %s..%s | %s | %s | %.1f %% |' % (
                name, fmt(p['x']['min']), fmt(p['x']['max']), fmt(p['y']['min']),
                fmt(p['y']['max']), fmt(p['z']['median']), fmt(p['m_mean']), 100 * p['m_spread']))
    if 'far' in results['steps']:
        grid = results['steps']['far']['fields']['sensor_mean_abs_b']
        lines += ['', '## Far step: mean abs B per sensor (board positions unknown)', '']
        for r in range(4):
            cells = ['S%-2d %8s' % (4 * r + c + 1, fmt(grid[4 * r + c])) for c in range(4)]
            lines.append('    ' + '  '.join(cells))
    text = '\n'.join(t['text'].strip() for s in results['steps'].values() for t in s['text'])
    if text:
        lines += ['', '## Text outside packets (possible firmware messages)', '',
                  '```', text[:TEXT_MAX_CHARS], '```']
    lines += ['', '## What to ask the firmware owner', '']
    lines += ['- ' + q for q in firmware_questions(results)]
    with open(os.path.join(folder, 'report.md'), 'w', encoding='utf-8') as handle:
        handle.write('\n'.join(lines) + '\n')
    write_json(os.path.join(folder, 'report.json'), results)
    return plain


# --------------------------------------------------------------------------
# Recording (the only code that touches the serial port, and it only reads)
# --------------------------------------------------------------------------

def import_serial():
    try:
        import serial
        import serial.tools.list_ports  # noqa: F401
    except ImportError:
        raise SystemExit('pyserial is missing. Install it with `pip install pyserial`, or run this '
                         'inside the colmag container (python3-serial is installed there).')
    return serial


def list_port_info():
    """Step 0: every serial port with its USB identity and /dev/serial/by-id names."""
    serial = import_serial()
    by_id = {}
    for link in glob.glob('/dev/serial/by-id/*') + glob.glob('/host/dev/serial/by-id/*'):
        by_id.setdefault(os.path.basename(os.path.realpath(link)), []).append(link)
    ports = []
    for p in serial.tools.list_ports.comports():
        if p.vid is None and re.search(r'/ttyS\d+$', p.device):
            continue  # the PC's own serial ports (often 32 of them), never the board
        ports.append({'device': p.device, 'description': p.description,
                      'manufacturer': p.manufacturer, 'product': p.product, 'vid': p.vid,
                      'pid': p.pid, 'serial_number': p.serial_number,
                      'by_id': by_id.get(os.path.basename(p.device), [])})
    listed = {os.path.basename(p['device']) for p in ports}
    for device in sorted(glob.glob('/host/dev/ttyACM*') + glob.glob('/host/dev/ttyUSB*')):
        if os.path.basename(device) not in listed:
            ports.append({'device': device, 'description': 'host device (no USB info here)',
                          'vid': None, 'by_id': by_id.get(os.path.basename(device), [])})
    return ports


def choose_port(ports, prompt):
    """The only ttyACM/ttyUSB port, or ask for a number."""
    boards = [p for p in ports if re.search(r'tty(ACM|USB)\d+$', p['device'])]
    if len(boards) == 1:
        return boards[0]['device']
    if not ports:
        raise SystemExit('No serial ports found. Is the board plugged in? Or pass --port.')
    for i, p in enumerate(ports, start=1):
        print('  %d: %s (%s)' % (i, p['device'], p.get('description')))
    while True:
        answer = prompt('Select port number: ').strip()
        if answer.isdigit() and 1 <= int(answer) <= len(ports):
            return ports[int(answer) - 1]['device']


def explain_port_error(exc):
    """The error text plus a hint for the usual causes."""
    text, errno = str(exc), getattr(exc, 'errno', None)
    if errno == 13 or 'Permission denied' in text:
        text += (' -> add yourself to the dialout group (sudo usermod -aG dialout $USER). It '
                 'only applies after you log in again; until then start the probe with '
                 'sg dialout -c "python3 tools/probe_board.py ...", or run inside the container')
    elif errno == 16 or 'busy' in text.lower():
        text += ' -> the port is busy: close the launcher / magnetometer_reader first'
    elif errno == 2 or 'No such file' in text:
        text += ' -> the port is gone: check the cable, or pick another port'
    return text


def open_serial_port(device, baudrate):
    """Open like magnetometer_reader.py: 8N1, pyserial defaults (pyserial drops stale input)."""
    serial = import_serial()
    try:
        return serial.Serial(port=device, baudrate=baudrate, timeout=READ_TIMEOUT_S)
    except (serial.SerialException, OSError) as exc:
        raise RuntimeError('Could not open %s: %s' % (device, explain_port_error(exc)))


def record_step(port, seconds, name, flush=False):
    """Read for `seconds`, keeping (monotonic ns, nbytes) per non-empty read. Never writes.

    With flush=True it first drops the bytes that piled up on this computer
    while the prompt waited for Enter: they belong to no step. That is a
    host-side flush, the same one pyserial does on every open; nothing is sent.
    """
    chunks, reads, stopped, stale = [], [], None, None
    start = time.monotonic_ns()
    end, shown = start + int(seconds * 1e9), None
    try:
        if flush:
            stale = port.in_waiting
            port.reset_input_buffer()
        while time.monotonic_ns() < end:
            # Take whatever is waiting, or block for one byte: each read then stamps
            # the moment its bytes arrived, not the moment a full packet-sized chunk did.
            data = port.read(port.in_waiting or 1)
            if data:
                reads.append((time.monotonic_ns(), len(data)))
                chunks.append(bytes(data))
            left = int((end - time.monotonic_ns()) / 1e9) + 1
            if left != shown:
                shown = left
                total = sum(n for _, n in reads)
                sys.stdout.write('\r  %s: %2d s left, %d bytes   ' % (name, left, total))
                sys.stdout.flush()
    except KeyboardInterrupt:
        stopped = 'ctrl-c'
    except OSError as exc:  # pyserial's SerialException is an OSError
        stopped = 'port error: %s' % explain_port_error(exc)
    print('')
    return {'data': b''.join(chunks), 'reads': reads, 'start_ns': start,
            'end_ns': time.monotonic_ns(), 'stopped': stopped, 'stale_bytes': stale}


def save_step(folder, name, rec):
    """<step>.bin with every byte, <step>_reads.csv with the arrival time of each read."""
    with open(os.path.join(folder, name + '.bin'), 'wb') as handle:
        handle.write(rec['data'])
    with open(os.path.join(folder, name + '_reads.csv'), 'w', newline='') as handle:
        writer = csv.writer(handle)
        writer.writerow(['t_ns', 'nbytes'])
        writer.writerows(rec['reads'])


def print_ports(ports):
    for p in ports:
        vid = '-'
        if p.get('vid') is not None:
            vid = '%04X:%04X' % (p['vid'], p.get('pid') or 0)
        print('  %s  %s  %s %s  VID:PID %s  serial %s  %s' % (
            p['device'], p.get('description'), p.get('manufacturer') or '',
            p.get('product') or '', vid, p.get('serial_number') or '-',
            ' '.join(p.get('by_id') or [])))


def run_probe(args, folder, prompt, open_port, list_ports):
    """Step 0 identify, then record each step on Enter. Ctrl-C keeps what was captured."""
    steps = STEPS[:1] if args.listen_only else STEPS
    print('MagPilot board probe. READ-ONLY: nothing is ever written to the board.')
    print('%d recording(s) after step 0, about %d s in total. Ctrl-C stops at any time and '
          'keeps what was recorded.' % (len(steps), sum(step[3] for step in steps)))
    print('\nStep 0 identify:')
    ports = list_ports()
    print_ports(ports)
    device = args.port or choose_port(ports, prompt)
    real = os.path.basename(os.path.realpath(device))      # a by-id link names its tty
    info = next((p for p in ports if os.path.basename(p['device']) == real), None)
    print('  -> using %s: %s' % (device, board_answer(info)['text']))
    try:
        os.makedirs(folder)
    except OSError as exc:
        raise SystemExit('Cannot write %s: %s; pass --output-dir somewhere writable.'
                         % (folder, exc))
    print('  Output: %s' % folder)
    meta = {'tool_version': TOOL_VERSION, 'python': sys.version, 'platform': platform.platform(),
            'argv': sys.argv, 'started_local': datetime.now().astimezone().isoformat(),
            'port': device, 'baudrate': args.baudrate, 'port_info': info, 'ports': ports,
            'openings': [], 'steps': []}
    write_json(os.path.join(folder, 'meta.json'), meta)
    port, open_ns = None, None
    try:
        for label, name, action, seconds, instruction in steps:
            if action == 'reopen' and port is not None:
                port.close()
                port = None
            print('\nStep %s %s (%d s): %s' % (label, name, seconds, instruction))
            prompt('  Press Enter to start > ')
            if action in ('open', 'reopen'):
                open_ns = time.monotonic_ns()
                port = open_port(device, args.baudrate)
                meta['openings'].append({'step': name, 'open_ns': open_ns})
            rec = record_step(port, seconds, name, flush=action == 'keep')
            save_step(folder, name, rec)
            meta['steps'].append({'name': name, 'label': label, 'start_ns': rec['start_ns'],
                                  'end_ns': rec['end_ns'], 'open_ns': open_ns,
                                  'opened_here': action != 'keep',
                                  'stale_bytes': rec['stale_bytes'], 'stopped': rec['stopped']})
            write_json(os.path.join(folder, 'meta.json'), meta)
            if rec['stopped']:
                print('  Stopped (%s); keeping what was captured.' % rec['stopped'])
                break
            if name == 'far':
                far, _ = analyze_step(rec['data'], rec['reads'], open_ns, True)
                if not far_stream_ok(far):
                    print('  Only %d valid packet(s) in %d bytes: %s.'
                          % (far['timing']['packets'], far['bytes'], WRONG_PORT_HINT))
                    break
    except (KeyboardInterrupt, EOFError) as exc:
        if isinstance(exc, EOFError):
            print('\n  Input closed (no keyboard): run with `docker exec -it`. '
                  'Keeping what was captured.')
        else:
            print('\n  Stopped with Ctrl-C; keeping what was captured.')
    except RuntimeError as exc:
        print('  ' + str(exc))
        meta['error'] = str(exc)
    finally:
        if port is not None:
            port.close()
        write_json(os.path.join(folder, 'meta.json'), meta)


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------

def build_parser():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--port', '-p', default=None,
                        help='Serial port (default: the only ttyACM/ttyUSB port, else ask)')
    parser.add_argument('--baudrate', '-b', type=int, default=921600)
    default_output = os.path.join(_ROOT, 'data_collection', 'board_probe')
    parser.add_argument('--output-dir', default=default_output,
                        help='Each run makes a YYYYmmdd_HHMMSS subfolder here')
    parser.add_argument('--listen-only', action='store_true',
                        help='Only steps 0 and 1: a 10 s check that the board streams')
    parser.add_argument('--replay', metavar='FOLDER', default=None,
                        help='No serial: re-analyse a saved probe folder and rewrite its report')
    return parser


def main(argv=None, prompt=input, open_port=open_serial_port, list_ports=list_port_info):
    args = build_parser().parse_args(argv)
    folder = args.replay
    if folder and not os.path.exists(os.path.join(folder, 'meta.json')):
        raise SystemExit('%s is not a probe folder (no meta.json).' % folder)
    if not folder:
        folder = os.path.join(args.output_dir, datetime.now().strftime('%Y%m%d_%H%M%S'))
        while os.path.exists(folder):
            folder += '_2'
        try:
            run_probe(args, folder, prompt, open_port, list_ports)
        except (KeyboardInterrupt, EOFError):
            pass
        if not os.path.exists(os.path.join(folder, 'meta.json')):
            print('\nStopped before anything was recorded.')
            return 1
    meta = read_json(os.path.join(folder, 'meta.json'))
    if meta.get('error') and not meta.get('steps'):
        print('\nNothing was recorded: %s' % meta['error'])
        return 1
    answers = write_report(analyze_folder(folder), folder)
    report = os.path.abspath(os.path.join(folder, 'report.md'))
    print('\nAnswers\n%s\n\nReport: %s' % (answers, report))
    return 1 if meta.get('error') else 0


if __name__ == '__main__':
    sys.exit(main())
