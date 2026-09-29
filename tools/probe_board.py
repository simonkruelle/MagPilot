#!/usr/bin/env python3
"""Probe the MagPilot sensor board and report what its firmware does.

The board (16 MLX90393 magnetometers, 4x4 on a 150 x 150 mm board) streams
218-byte packets over USB serial, but the firmware source is not available.
This script walks you through six short recordings (stylus far away, still,
sweeping, pressed onto one sensor, then a port close/reopen) and answers: which
board it is, the real sample rate, whether host timestamps can be trusted, the
field units, whether a baseline is subtracted, whether opening the port resets
or recalibrates the board, whether sensors saturate, and what the pose means.

READ-ONLY: the script never writes to the serial port and never sets DTR, RTS
or break itself. It opens the port with pyserial's defaults, like
magnetometer_reader.py, so the operating system raises DTR/RTS on open as it
always does; the effect of that is one of the things being measured.

Each run writes one folder named by local time (data_collection/ is gitignored):

    <output_dir>/<YYYYmmdd_HHMMSS>/meta.json         ports, open times, steps
    <output_dir>/<YYYYmmdd_HHMMSS>/<step>.bin        every raw byte received
    <output_dir>/<YYYYmmdd_HHMMSS>/<step>_reads.csv  t_ns,nbytes of each read
    <output_dir>/<YYYYmmdd_HHMMSS>/report.md         answers in plain English
    <output_dir>/<YYYYmmdd_HHMMSS>/report.json       all numbers

Close the launcher and magnetometer_reader first: two programs reading one port
each get only part of the stream.

Usage inside the container (-it is needed for the Enter prompts):
  docker exec -it colmag_simon bash -lc 'cd /colmag && python3 tools/probe_board.py --port /dev/ttyACM0'
  (use /host/dev/ttyACM0 if the board was plugged in after the container started)
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

from colmag.control_mapping import MAGNET_HEIGHT_SENSOR_BIAS_M  # noqa: E402

TOOL_VERSION = '1.0'

# Packet layout shared with magnetometer_reader.py.
PACKET_SIZE = 218
PACKET_HEADER = 0xAA
PACKET_TAIL = 0xBB
PACKET_STRUCT = struct.Struct('<54f')
N_SENSORS = 16
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
TEXT_MIN_CHARS = 4              # shorter printable runs are usually just float bytes
TEXT_MAX_CHARS = 500            # keep the report readable
GAP_EDGES_MS = (1, 5, 20, 50, 100)
BATCHED_FRACTION = 0.2          # more <1 ms gaps than this: timestamps show USB batching
BASELINE_NOISE_RATIO = 5.0      # far |B| under 5x the noise: Earth's field is not in the data
CLIP_MIN_RUN = 3                # an extreme repeated this often in a row looks pinned
CLIP_MIN_FRACTION = 0.5         # ...and is at least half the largest value seen anywhere
CLIP_MIN_NOISE = 20.0           # ...and far above the far-step noise
CLIP_MIN_RANGE = 0.25           # ...while the channel moved (still magnets are not clipping)
WRAP_MIN_FRACTION = 0.5         # sign-flip jump larger than half the channel's range
UNIT_MOMENT_TOL = 0.05          # |m| within 5 % of 1 is a unit vector
STEADY_SPREAD = 0.02            # |m| relative spread under 2 % is constant
FITTED_SPREAD = 0.10            # over 10 % means the strength is fitted (or noisy)
OPEN_SHIFT_SIGMA = 5.0          # a channel moved at reopen if it shifted by more than 5 noise
OPEN_SHIFT_FRACTION = 0.25      # ...and at least a quarter of channels did
RESTART_DELAY_S = 0.5           # first packet later than this after open suggests a reboot
RESTART_TEXT_S = 3.0            # firmware text this soon after open suggests a boot banner

# (unit, low, high, factor to uT): wide windows around Earth's 25-65 uT (NOAA),
# widened for steel furniture and the board's own electronics.
UNIT_WINDOWS = (
    ('tesla', 1.5e-5, 1.0e-4, 1e6), ('mT', 0.015, 0.1, 1e3),
    ('gauss', 0.15, 1.0, 100.0), ('uT', 15.0, 100.0, 1.0),
    ('mG', 150.0, 1000.0, 0.1), ('nT', 15000.0, 100000.0, 1e-3),
)
# MLX90393 full scale in uT (sensitivity x code range, datasheet Rev 012, HALLCONF 0xC).
FULL_SCALE_UT = (
    ('GAIN_SEL 7, RES 0 (Adafruit default)', 4922, 7930),
    ('GAIN_SEL 5, RES 0', 8204, 13216),
    ('GAIN_SEL 0, RES 0', 24612, 39649),
    ('GAIN_SEL 0, RES 1', 49225, 79299),
    ('GAIN_SEL 0, RES 2/3 (largest)', 66098, 106480),
)
HALL_SATURATION_UT = 50000      # datasheet BSAT onset; readings above are not trustworthy

USB_VENDORS = {
    0x2341: 'Arduino', 0x2A03: 'Arduino', 0x16C0: 'Teensy (PJRC)',
    0x303A: 'Espressif native USB (ESP32-S2/S3/C3)',
    0x10C4: 'Silicon Labs CP210x USB-UART bridge', 0x1A86: 'WCH CH340 USB-UART bridge',
    0x0403: 'FTDI USB-UART bridge', 0x0483: 'STMicroelectronics (STM32)',
    0x2E8A: 'Raspberry Pi RP2040', 0x239A: 'Adafruit', 0x1B4F: 'SparkFun',
}
BRIDGE_VIDS = (0x10C4, 0x1A86, 0x0403)

# (name, port action, seconds, instruction). 'open' and 'reopen' open the port
# just before recording; 'keep' records on the port that is already open.
STEPS = (
    ('far', 'open', FAR_S,
     'Put the stylus at least 1 m away from the board.'),
    ('still', 'keep', STILL_S,
     'Hold the stylus upright and still on the cover over the centre of the board.'),
    ('sweep', 'keep', SWEEP_S,
     'Move the stylus slowly over the whole writing area, touching the cover, '
     'and tilt it now and then.'),
    ('closest', 'keep', CLOSEST_S,
     'Press the stylus onto the cover right above one sensor, then tilt it slowly in all directions.'),
    ('reopen_magnet', 'reopen', REOPEN_MAGNET_S,
     'The port is closed. Put the stylus upright on the cover over the centre and leave it there.'),
    ('reopen_far', 'keep', REOPEN_FAR_S,
     'Now take the stylus at least 1 m away.'),
)
MAGNET_STEPS = ('still', 'sweep', 'closest', 'reopen_magnet')

CHANNELS = ['Sensor%d_%s' % (i + 1, axis) for i in range(N_SENSORS) for axis in ('Bx', 'By', 'Bz')]
CHANNELS += ['Pose_x', 'Pose_y', 'Pose_z', 'Pose_mx', 'Pose_my', 'Pose_mz']
POSE = {name: 48 + i for i, name in enumerate(('x', 'y', 'z', 'mx', 'my', 'mz'))}


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
    return [mean_std([row[c] for row in rows])[0] for c in range(48)]


def fmt(value, digits=4):
    """Short number for the report ('n/a' for None)."""
    return 'n/a' if value is None else '%.*g' % (digits, value)


# --------------------------------------------------------------------------
# Analysis: pure functions over bytes, reads and open times
# --------------------------------------------------------------------------

def split_packets(data):
    """Split a byte stream exactly like magnetometer_reader.py's read loop.

    Find 0xAA, need 218 bytes, require 0xBB at index 217, else drop one byte
    and search again. Returns packets as (index of last byte, 54 floats), the
    resync count, the skipped byte ranges and the offset of the first packet.
    """
    packets, skipped, resyncs, pos, n = [], [], 0, 0, len(data)

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
        if data[last] != PACKET_TAIL:
            resyncs += 1
            skip(pos, pos + 1)
            pos += 1
            continue
        packets.append((last, PACKET_STRUCT.unpack(bytes(data[pos + 1:last]))))
        pos += PACKET_SIZE
    return {'packets': packets, 'resyncs': resyncs, 'skipped': skipped,
            'skipped_bytes': sum(e - s for s, e in skipped),
            'bytes_before_first_packet': packets[0][0] - (PACKET_SIZE - 1) if packets else n,
            'leftover_bytes': n - pos}


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


def find_text(data, skipped):
    """Printable ASCII runs inside skipped bytes: boot banners, versions, debug prints."""
    pattern = re.compile(rb'[\t\r\n\x20-\x7e]{%d,}' % TEXT_MIN_CHARS)
    runs = []
    for start, end in skipped:
        for match in pattern.finditer(bytes(data[start:end])):
            runs.append((start + match.start(), match.group().decode('ascii')))
    return runs


def timing_stats(times_ns, read_indices):
    """Rate, packets per 1-s window, gap buckets and packets per read."""
    n = len(times_ns)
    out = {'packets': n}
    if n < 2 or times_ns[-1] == times_ns[0]:
        return out
    span_s = (times_ns[-1] - times_ns[0]) / 1e9
    out['rate_hz'] = (n - 1) / span_s
    windows = Counter(int((t - times_ns[0]) // 1e9) for t in times_ns)
    full = [windows.get(i, 0) for i in range(int(span_s))]      # complete windows only
    if full:
        out['per_second'] = {'min': min(full), 'median': statistics.median(full), 'max': max(full)}
    gaps = [(b - a) / 1e6 for a, b in zip(times_ns, times_ns[1:])]
    edges = (0.0,) + GAP_EDGES_MS + (float('inf'),)
    labels = ['<1 ms'] + ['%d-%d ms' % pair for pair in zip(GAP_EDGES_MS, GAP_EDGES_MS[1:])] + ['>=100 ms']
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
        if c < 48:
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
    stds = [ch['std'] for ch in list(out['channels'].values())[:48] if ch]
    out['median_std'] = statistics.median(stds) if stds else None
    flat = finite([v for row in rows for v in row[:48]])
    out['all_integer'] = bool(flat) and any(flat) and all(v == int(v) for v in flat)
    return out


def guess_units(far):
    """Guess the field unit from the far step's median |B| (Earth's field is 25-65 uT)."""
    b, noise = far.get('median_abs_b'), far.get('median_std') or 0.0
    out = {'median_abs_b': b, 'noise': noise, 'unit': None, 'to_ut': None, 'baseline': None,
           'confidence': 'unclear', 'text': 'no far-step packets'}
    if b is None:
        return out
    evidence = 'far-step median |B| = %s, noise (median std) = %s' % (fmt(b), fmt(noise))
    if far.get('all_integer'):
        out.update(unit='counts', confidence='likely',
                   text='raw sensor counts: all values are integers (%s; Earth would be about '
                        '167-433 counts at GAIN_SEL 7, RES 0)' % evidence)
    elif b < BASELINE_NOISE_RATIO * noise or b == 0.0:
        out.update(baseline=True, confidence='likely',
                   text='a baseline is probably subtracted, so the units cannot be read from '
                        'the Earth field (%s)' % evidence)
    else:
        out['text'] = "no unit puts the far |B| near Earth's 25-65 uT (%s)" % evidence
        for unit, low, high, factor in UNIT_WINDOWS:
            if low <= b <= high:
                out.update(unit=unit, to_ut=factor, baseline=False, confidence='likely',
                           text="%s: far from the magnet |B| is %s uT, inside Earth's 25-65 uT (%s)"
                                % (unit, fmt(b * factor, 3), evidence))
                break
    return out


def longest_run(values, target):
    best = run = 0
    for v in values:
        run = run + 1 if v == target else 0
        best = max(best, run)
    return best


def detect_saturation(magnet_rows, far):
    """Look for pinned extremes (clipping), sign-flip jumps (wrap-around) and shared limits.

    A channel counts as clipped only if its extreme repeats exactly in a row,
    is large (vs every channel and vs the far-step noise) and the channel
    moved a lot in that step, so a still magnet with quantised readings is not
    flagged.
    """
    out = {'global_max_abs': None, 'max_abs_xy': None, 'max_abs_z': None,
           'clipped': [], 'wraps': [], 'shared_limits': []}
    series = {step: [[row[c] for row in rows] for c in range(48)]
              for step, rows in magnet_rows.items() if rows}
    if not series:
        return out
    every = [finite([v for cols in series.values() for v in cols[c]]) or [0.0] for c in range(48)]
    chan_max = [max(abs(v) for v in values) for values in every]
    chan_span = [max(values) - min(values) for values in every]
    top = max(chan_max)
    out.update(global_max_abs=top, max_abs_xy=max(m for c, m in enumerate(chan_max) if c % 3 != 2),
               max_abs_z=max(chan_max[2::3]), channel_max_abs=dict(zip(CHANNELS, chan_max)))
    far_channels = far.get('channels') or {}
    floor = max(far.get('median_std') or 0.0, 1e-9)
    for step, cols in series.items():
        for c, values in enumerate(cols):
            good = finite(values)
            if not good:
                continue
            extreme = max(good, key=abs)
            run = longest_run(values, extreme)
            noise = max((far_channels.get(CHANNELS[c]) or {}).get('std') or 0.0, floor)
            moved = max(good) - min(good) >= CLIP_MIN_RANGE * abs(extreme)
            if (run >= CLIP_MIN_RUN and moved and abs(extreme) >= CLIP_MIN_FRACTION * top
                    and abs(extreme) >= CLIP_MIN_NOISE * noise):
                out['clipped'].append({'step': step, 'channel': CHANNELS[c], 'value': extreme, 'run': run})
            for i, (a, b) in enumerate(zip(values, values[1:])):
                if (a * b < 0 and abs(a - b) > WRAP_MIN_FRACTION * chan_span[c]
                        and min(abs(a), abs(b)) >= WRAP_MIN_FRACTION * chan_max[c]):
                    out['wraps'].append({'step': step, 'channel': CHANNELS[c], 'sample': i + 1,
                                         'from': a, 'to': b})
    # Shared limit: several channels stop at the exact same magnet-sized magnitude.
    magnet_sized = max(CLIP_MIN_FRACTION * top, CLIP_MIN_NOISE * floor, 2.0 * (far.get('median_abs_b') or 0.0))
    groups = Counter(m for m in chan_max if m > 0 and m >= magnet_sized)
    out['shared_limits'] = [{'magnitude': m, 'channels': [CHANNELS[c] for c in range(48) if chan_max[c] == m]}
                            for m, count in groups.items() if count >= 2]
    return out


def full_scale_note(sat, units):
    """Compare the largest readings with MLX90393 full-scale values, without overclaiming."""
    xy, z = sat.get('max_abs_xy'), sat.get('max_abs_z')
    if xy is None:
        return 'no magnet-step data'
    if units.get('unit') == 'counts':
        return ('largest |value| %s counts (int16 limit at RES 0/1 is 32768; RES 2 spans +/-22000, '
                'RES 3 +/-11000)' % fmt(max(xy, z)))
    if not units.get('to_ut'):
        return 'largest |value| %s (units unknown, so no full-scale comparison)' % fmt(max(xy, z))
    xy_ut, z_ut = xy * units['to_ut'], z * units['to_ut']
    over = [name for name, fs_xy, fs_z in FULL_SCALE_UT if xy_ut > fs_xy or z_ut > fs_z]
    text = 'largest |Bxy| = %s uT, |Bz| = %s uT' % (fmt(xy_ut, 3), fmt(z_ut, 3))
    if not over:
        return text + '; below the smallest MLX90393 full scale (4922/7930 uT), so no setting clips there'
    text += ('; above the full scale of %s, so the firmware does not use those settings '
             '(or those readings are clipped)' % '; '.join(over))
    if max(xy_ut, z_ut) > HALL_SATURATION_UT:
        text += '; above the 50 mT Hall-plate saturation onset, so those readings are not trustworthy'
    return text


def pose_stats(rows):
    """Pose ranges and |m| = sqrt(mx^2+my^2+mz^2) mean and relative spread."""
    out = {}
    for axis in ('x', 'y', 'z'):
        good = finite([row[POSE[axis]] for row in rows])
        out[axis] = {'min': min(good), 'max': max(good), 'median': statistics.median(good)} if good else None
    moments = [math.sqrt(row[POSE['mx']] ** 2 + row[POSE['my']] ** 2 + row[POSE['mz']] ** 2) for row in rows]
    m, s = mean_std(moments)
    out['m_mean'], out['m_spread'] = m, (s / m if m else None)
    return out


def guess_pose_units(sweep_rows):
    """Metres if the sweep's 95th-percentile |x|,|y| is under 0.2; mm if it is tens of units."""
    xy = sorted(abs(v) for row in sweep_rows for v in finite([row[POSE['x']], row[POSE['y']]]))
    if not xy:
        return {'unit': None, 'confidence': 'unclear', 'text': 'no sweep data'}
    p95 = xy[int(0.95 * (len(xy) - 1))]
    outside = sum(v > POSE_LIMIT_M for v in xy) / len(xy)
    evidence = ("sweep 95th-percentile |x|,|y| = %s, %.0f %% of values beyond the reader's 0.05 m window"
                % (fmt(p95), 100 * outside))
    if p95 == 0.0:
        unit, conf, text = None, 'measured', 'pose is all zero: the board does not compute it'
    elif p95 <= 0.2:
        unit, conf, text = 'm', 'likely', 'metres from the board centre'
    elif 5.0 <= p95 <= 500.0:
        unit, conf, text = 'mm', 'likely', 'millimetres'
    else:
        unit, conf, text = None, 'unclear', 'neither metres nor mm'
    return {'unit': unit, 'confidence': conf, 'p95': p95, 'text': '%s (%s)' % (text, evidence)}


def interpret_moment(magnet_rows):
    """Unit direction vector, fixed strength or fitted strength, from |m| over the magnet steps."""
    stats = pose_stats([row for rows in magnet_rows.values() for row in rows])
    m, spread = stats['m_mean'], stats['m_spread']
    if m is None or spread is None:
        return {'kind': None, 'confidence': 'unclear', 'text': 'no magnet-step data'}
    if abs(m - 1.0) < UNIT_MOMENT_TOL and spread < STEADY_SPREAD:
        kind, conf, text = 'unit', 'measured', ('a unit direction vector (the dipole axis), '
                                                'as colmag/control_mapping.py assumes')
    elif spread < STEADY_SPREAD:
        kind, conf, text = 'fixed', 'measured', ('a fixed-strength moment (direction times a constant); '
                                                 'control_mapping.py normalises it, so angles are right')
    elif spread > FITTED_SPREAD:
        kind, conf, text = 'fitted', 'likely', ('a fitted moment whose strength changes (or a noisy fit); '
                                                'only its direction is used')
    else:
        kind, conf, text = 'unclear', 'unclear', 'nearly constant length, but not clearly a unit vector'
    evidence = '|m| mean %s, relative spread %.1f %%' % (fmt(m), 100 * spread)
    return {'kind': kind, 'confidence': conf, 'm_mean': m, 'm_spread': spread,
            'text': '%s (%s)' % (text, evidence)}


def port_open_effect(far_rows, still_rows, reopen_far_rows):
    """Did reopening the port (with the stylus on the board) move the far-field readings?

    If the board captures its baseline at open, the stylus field at that
    moment is baked in, so after the reopen the far readings shift by minus
    that field. The still step had the stylus at the same place, so it gives
    the expected shift: -(still - far). (reopen_magnet itself cannot: with a
    re-captured baseline it would read about zero.)
    """
    if not far_rows or not reopen_far_rows:
        return {'result': None, 'confidence': 'unclear', 'text': 'no reopen data (run all steps to answer this)'}
    far_mean, reopen_mean = column_means(far_rows), column_means(reopen_far_rows)
    stds = [mean_std([row[c] for row in far_rows])[1] or 0.0 for c in range(48)]
    floor = max(statistics.median(stds), 1e-3 * statistics.median([abs(v or 0.0) for v in far_mean]), 1e-9)
    shift = [(r or 0.0) - (f or 0.0) for r, f in zip(reopen_mean, far_mean)]
    sigmas = [abs(s) / max(sd, floor) for s, sd in zip(shift, stds)]
    moved = sum(v > OPEN_SHIFT_SIGMA for v in sigmas) / 48.0
    out = {'shift_sigma_median': statistics.median(sigmas), 'shift_sigma_max': max(sigmas),
           'moved_fraction': moved}
    evidence = 'reopen_far vs far: median %.1f, max %.1f noise units, %.0f %% of channels moved' % (
        out['shift_sigma_median'], out['shift_sigma_max'], 100 * moved)
    if moved < OPEN_SHIFT_FRACTION:
        out.update(result='none', confidence='measured',
                   text='no baseline is re-captured at port open (%s)' % evidence)
        return out
    expected = [0.0] * 48
    if still_rows:
        expected = [-((s or 0.0) - (f or 0.0)) for s, f in zip(column_means(still_rows), far_mean)]
    dot = sum(a * b for a, b in zip(shift, expected))
    norm = math.sqrt(sum(a * a for a in shift) * sum(b * b for b in expected))
    out['correlation'] = dot / norm if norm else 0.0
    out['ratio'] = dot / sum(b * b for b in expected) if any(expected) else 0.0
    evidence += ', correlation with -(still - far) %.2f, size ratio %.2f' % (out['correlation'], out['ratio'])
    if out['correlation'] > 0.8 and 0.5 <= out['ratio'] <= 1.5:
        out.update(result='baseline', confidence='measured',
                   text='the board captures its baseline when the port opens, so always open the port '
                        'with the stylus far away (%s)' % evidence)
    else:
        out.update(result='other', confidence='unclear',
                   text='readings changed after the reopen, but not like a re-captured baseline; maybe '
                        'drift, or the stylus was not fully away (%s)' % evidence)
    return out


def restart_evidence(steps):
    """Firmware text right after opening, or a slow first packet, suggests a reboot on open."""
    signs, delays = [], []
    for name, step in steps.items():
        if not step.get('opened_here'):
            continue
        delay = step['timing'].get('open_to_first_packet_s')
        if delay is not None:
            delays.append('%s %.3f s' % (name, delay))
            if delay > RESTART_DELAY_S:
                signs.append('first packet %.2f s after opening in %s' % (delay, name))
        if any(t['after_open_s'] is not None and t['after_open_s'] < RESTART_TEXT_S for t in step['text']):
            signs.append('firmware text right after opening in %s' % name)
    if not delays:
        return {'restart': None, 'confidence': 'unclear', 'text': 'no packets after an open'}
    if signs:
        return {'restart': True, 'confidence': 'likely', 'text': 'probable restart on open: ' + '; '.join(signs)}
    return {'restart': False, 'confidence': 'likely',
            'text': 'no sign of a restart on open (first packet after: %s; no text)' % ', '.join(delays)}


def board_answer(info):
    """Board family from the USB vendor id of the chosen port."""
    if not info or info.get('vid') is None:
        return {'confidence': 'unclear',
                'text': 'no USB vendor id for this port (inside the container, run step 0 on the host too)'}
    vid, pid = info['vid'], info.get('pid') or 0
    family = USB_VENDORS.get(vid, 'unknown vendor')
    text = '%s, VID:PID %04X:%04X -> %s' % (info.get('description') or info.get('device'), vid, pid, family)
    if vid in BRIDGE_VIDS:
        text += '; a USB-UART bridge usually toggles DTR/RTS on open and may reset the microcontroller'
    return {'confidence': 'likely' if vid in USB_VENDORS else 'unclear', 'text': text, 'family': family}


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
    texts = []
    for offset, text in find_text(data, split['skipped']):
        after = (arrival_of([offset], reads)[0][0] - open_ns) / 1e9 if since_open else None
        texts.append({'offset': offset, 'text': text, 'after_open_s': after})
    result = {'bytes': len(data), 'reads': len(reads), 'opened_here': opened_here, 'timing': timing,
              'text': texts, 'fields': field_stats(rows), 'pose': pose_stats(rows) if rows else None}
    for key in ('resyncs', 'skipped_bytes', 'bytes_before_first_packet', 'leftover_bytes'):
        result[key] = split[key]
    return result, rows


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
        steps[name], rows[name] = analyze_step(data, reads, step.get('open_ns'), step.get('opened_here', False))
        steps[name]['duration_s'] = (step['end_ns'] - step['start_ns']) / 1e9
        steps[name]['stopped'] = step.get('stopped')
    far = steps['far']['fields'] if 'far' in steps else {}
    magnet_rows = {name: rows[name] for name in MAGNET_STEPS if name in rows}
    units = guess_units(far)
    saturation = detect_saturation(magnet_rows, far)
    saturation['full_scale'] = full_scale_note(saturation, units)
    results = {
        'tool_version': TOOL_VERSION, 'folder': os.path.abspath(folder), 'meta': meta, 'steps': steps,
        'board': board_answer(meta.get('port_info')), 'units': units, 'saturation': saturation,
        'pose_units': guess_pose_units(rows.get('sweep', [])), 'moment': interpret_moment(magnet_rows),
        'port_open': port_open_effect(rows.get('far'), rows.get('still'), rows.get('reopen_far')),
        'restart': restart_evidence(steps),
    }
    results['answers'] = build_answers(results)
    return results


# --------------------------------------------------------------------------
# Answers and report
# --------------------------------------------------------------------------

def build_answers(r):
    """One (question, answer with evidence, confidence) per question."""
    answers = [('Board', r['board']['text'], r['board']['confidence'])]

    far = r['steps'].get('far', {}).get('timing', {})
    if far.get('rate_hz'):
        windows = far.get('per_second')
        per_window = '; per 1-s window min/median/max %s/%s/%s' % (
            windows['min'], windows['median'], windows['max']) if windows else ''
        answers.append(('Sample rate', '%.1f packets/s in the far step (%d packets%s; the link carries '
                        'at most ~422/s)' % (far['rate_hz'], far['packets'], per_window), 'measured'))
        batched = far['gap_fractions']['<1 ms']
        usable = batched <= BATCHED_FRACTION
        answers.append(('Host timing usable for speed',
                        '%s: %.0f %% of packets arrive <1 ms after the previous one, i.e. in bursts that USB '
                        'hands over together (median %s packets per read), max gap %.1f ms%s'
                        % ('yes' if usable else 'no', 100 * batched, fmt(far['median_packets_per_read'], 3),
                           far['max_gap_ms'], '' if usable else '; use the packet count / mean rate instead'),
                        'measured'))
    else:
        answers.append(('Sample rate', 'no packets in the far step: wrong port or baud rate, or another '
                        'program is reading the port (close the launcher / magnetometer_reader first)', 'unclear'))

    units, port_open = r['units'], r['port_open']
    answers.append(('Units', units['text'], units['confidence']))
    if port_open.get('result') == 'baseline':
        answers.append(('Baseline subtracted', 'yes, captured when the port opens (see next line)', 'measured'))
    elif units.get('baseline') is True:
        answers.append(('Baseline subtracted', 'probably: far |B| is at the noise level', 'likely'))
    elif units.get('baseline') is False:
        answers.append(('Baseline subtracted', 'no: far |B| matches the Earth field', 'likely'))
    else:
        answers.append(('Baseline subtracted', 'cannot tell from these numbers', 'unclear'))
    answers.append(('Port open resets or recalibrates', '%s; %s' % (r['restart']['text'], port_open['text']),
                    port_open['confidence'] if port_open.get('result') else r['restart']['confidence']))

    sat = r['saturation']
    if sat['global_max_abs'] is None:
        answers.append(('Saturation', 'no magnet-step data', 'unclear'))
    else:
        found = ['%s pinned at %s for %d samples in %s' % (c['channel'], fmt(c['value']), c['run'], c['step'])
                 for c in sat['clipped'][:4]]
        found += ['%s jumps %s -> %s in %s' % (w['channel'], fmt(w['from']), fmt(w['to']), w['step'])
                  for w in sat['wraps'][:4]]
        text = ('yes: ' + '; '.join(found)) if found else 'no clipped or wrapped channels found'
        for group in sat['shared_limits'][:2]:
            text += '; %d channels share the same largest |value| %s (a common full-scale limit?)' % (
                len(group['channels']), fmt(group['magnitude']))
        answers.append(('Saturation', '%s; %s' % (text, sat['full_scale']), 'likely' if found else 'measured'))

    pose_text = r['pose_units']['text']
    still = (r['steps'].get('still') or {}).get('pose') or {}
    if still.get('z') and r['pose_units'].get('unit') == 'm':
        z = still['z']['median']
        pose_text += '; Pose_z on the cover = %s m (control_mapping.py subtracts %s m -> %s m)' % (
            fmt(z, 3), MAGNET_HEIGHT_SENSOR_BIAS_M, fmt(max(0.007, abs(z) - MAGNET_HEIGHT_SENSOR_BIAS_M), 3))
    answers.append(('Pose units', pose_text, r['pose_units']['confidence']))
    answers.append(('What mx/my/mz mean', r['moment']['text'], r['moment']['confidence']))
    return answers


def firmware_questions(r):
    """Only what the probe could not answer."""
    questions = ['Which MLX90393 settings (GAIN_SEL, RES, OSR, DIG_FILT, HALLCONF) and which '
                 'sensitivity table the firmware uses.',
                 'Which sensor number sits where on the board (the packets only say Sensor1..16).']
    if r['units']['confidence'] != 'measured':
        questions.append('The exact field unit and scale factor (the probe can only guess: %s).'
                         % (r['units']['unit'] or 'unclear'))
    if not r['saturation']['clipped'] and not r['saturation']['wraps']:
        questions.append('What the firmware sends when a sensor overflows (clip, wrap, or an error value).')
    if r['port_open'].get('result') in (None, 'other'):
        questions.append('Whether the board takes a baseline or restarts when the port opens.')
    if 'unclear' in (r['moment']['confidence'], r['pose_units']['confidence']):
        questions.append('How Pose_x..Pose_mz are computed and in which units.')
    return questions


def write_report(results, folder):
    """Write report.md and report.json; return the Answers text."""
    answers = '\n'.join('- **%s:** %s (%s)' % a for a in results['answers'])
    meta = results['meta']
    lines = ['# Board probe %s' % os.path.basename(results['folder']), '',
             'Recorded %s on %s at %s baud (probe version %s).' % (
                 meta.get('started_local'), meta.get('port'), meta.get('baudrate'), results['tool_version']),
             '', '## Answers', '', answers, '', '## Steps', '',
             '| step | s | packets | rate /s | <1 ms gaps | resyncs | skipped B | NaN/inf | zeros '
             '| median abs B | noise |',
             '|---|---|---|---|---|---|---|---|---|---|---|']
    for name, s in results['steps'].items():
        t, f = s['timing'], s['fields']
        batched = '%.0f %%' % (100 * t['gap_fractions']['<1 ms']) if 'gap_fractions' in t else 'n/a'
        lines.append('| %s | %.1f | %d | %s | %s | %d | %d | %d | %d | %s | %s |' % (
            name + (' (stopped)' if s.get('stopped') else ''), s['duration_s'], t['packets'],
            fmt(t.get('rate_hz')), batched, s['resyncs'], s['skipped_bytes'], f['nonfinite'],
            f['exact_zeros'], fmt(f['median_abs_b']), fmt(f['median_std'])))
    lines += ['', '## Pose per step', '',
              '| step | x min..max | y min..max | z median | abs m mean | abs m spread |',
              '|---|---|---|---|---|---|']
    for name, s in results['steps'].items():
        p = s['pose']
        if p and p['x'] and p['m_spread'] is not None:
            lines.append('| %s | %s..%s | %s..%s | %s | %s | %.1f %% |' % (
                name, fmt(p['x']['min']), fmt(p['x']['max']), fmt(p['y']['min']), fmt(p['y']['max']),
                fmt(p['z']['median']), fmt(p['m_mean']), 100 * p['m_spread']))
    if 'far' in results['steps']:
        grid = results['steps']['far']['fields']['sensor_mean_abs_b']
        lines += ['', '## Far step: mean abs B per sensor (board positions unknown)', '']
        lines += ['    ' + '  '.join('S%-2d %8s' % (4 * r + c + 1, fmt(grid[4 * r + c])) for c in range(4))
                  for r in range(4)]
    text = '\n'.join(t['text'].strip() for s in results['steps'].values() for t in s['text'])
    if text:
        lines += ['', '## Text outside packets (possible firmware messages)', '',
                  '```', text[:TEXT_MAX_CHARS], '```']
    lines += ['', '## What to ask the firmware owner', ''] + ['- ' + q for q in firmware_questions(results)]
    with open(os.path.join(folder, 'report.md'), 'w', encoding='utf-8') as handle:
        handle.write('\n'.join(lines) + '\n')
    write_json(os.path.join(folder, 'report.json'), results)
    return answers


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
        ports.append({'device': p.device, 'description': p.description, 'manufacturer': p.manufacturer,
                      'product': p.product, 'vid': p.vid, 'pid': p.pid, 'serial_number': p.serial_number,
                      'by_id': by_id.get(os.path.basename(p.device), [])})
    listed = {os.path.basename(p['device']) for p in ports}
    for device in sorted(glob.glob('/host/dev/ttyACM*') + glob.glob('/host/dev/ttyUSB*')):
        if os.path.basename(device) not in listed:
            ports.append({'device': device, 'description': 'host device (no USB info here)', 'vid': None})
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
        text += (' -> add yourself to the dialout group (sudo usermod -aG dialout $USER, then log in '
                 'again), or run inside the container')
    elif errno == 16 or 'busy' in text.lower():
        text += ' -> the port is busy: close the launcher / magnetometer_reader first'
    elif errno == 2 or 'No such file' in text:
        text += ' -> the port is gone: check the cable, or pick another port'
    return text


def open_serial_port(device, baudrate):
    """Open like magnetometer_reader.py: 8N1, pyserial defaults (pyserial drops stale input on open)."""
    serial = import_serial()
    try:
        return serial.Serial(port=device, baudrate=baudrate, timeout=READ_TIMEOUT_S)
    except (serial.SerialException, OSError) as exc:
        raise RuntimeError('Could not open %s: %s' % (device, explain_port_error(exc)))


def record_step(port, seconds, name):
    """Read for `seconds`, keeping (monotonic ns, nbytes) per non-empty read. Never writes."""
    chunks, reads, stopped = [], [], None
    start = time.monotonic_ns()
    end, shown = start + int(seconds * 1e9), None
    try:
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
                sys.stdout.write('\r  %s: %2d s left, %d bytes   ' % (name, left, sum(n for _, n in reads)))
                sys.stdout.flush()
    except KeyboardInterrupt:
        stopped = 'ctrl-c'
    except OSError as exc:  # pyserial's SerialException is an OSError
        stopped = 'port error: %s' % explain_port_error(exc)
    print('')
    return {'data': b''.join(chunks), 'reads': reads, 'start_ns': start, 'end_ns': time.monotonic_ns(),
            'stopped': stopped}


def read_json(path):
    with open(path, encoding='utf-8') as handle:
        return json.load(handle)


def write_json(path, payload):
    """Atomic JSON write (write .tmp, then replace), like record_tracking_error.py."""
    with open(path + '.tmp', 'w', encoding='utf-8') as handle:
        json.dump(payload, handle, indent=2, default=str)
        handle.write('\n')
    os.replace(path + '.tmp', path)


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
        vid = '%04X:%04X' % (p['vid'], p.get('pid') or 0) if p.get('vid') is not None else '-'
        print('  %s  %s  %s %s  VID:PID %s  serial %s  %s' % (
            p['device'], p.get('description'), p.get('manufacturer') or '', p.get('product') or '', vid,
            p.get('serial_number') or '-', ' '.join(p.get('by_id') or [])))


def run_probe(args, folder, prompt, open_port, list_ports):
    """Step 0 identify, then record each step on Enter. Ctrl-C keeps what was captured."""
    print('MagPilot board probe. READ-ONLY: nothing is ever written to the board.')
    print('\nStep 0 identify:')
    ports = list_ports()
    print_ports(ports)
    device = args.port or choose_port(ports, prompt)
    info = next((p for p in ports if os.path.basename(p['device']) == os.path.basename(device)), None)
    print('  -> using %s: %s' % (device, board_answer(info)['text']))
    os.makedirs(folder)
    print('  Output: %s' % folder)
    meta = {'tool_version': TOOL_VERSION, 'python': sys.version, 'platform': platform.platform(),
            'argv': sys.argv, 'started_local': datetime.now().astimezone().isoformat(), 'port': device,
            'baudrate': args.baudrate, 'port_info': info, 'ports': ports, 'openings': [], 'steps': []}
    write_json(os.path.join(folder, 'meta.json'), meta)
    port, open_ns = None, None
    try:
        for number, (name, action, seconds, instruction) in enumerate(STEPS[:1] if args.listen_only else STEPS, 1):
            if action == 'reopen' and port is not None:
                port.close()
                port = None
            print('\nStep %d %s (%d s): %s' % (min(number, 5), name, seconds, instruction))
            prompt('  Press Enter to start > ')
            if action in ('open', 'reopen'):
                open_ns = time.monotonic_ns()
                port = open_port(device, args.baudrate)
                meta['openings'].append({'step': name, 'open_ns': open_ns})
            rec = record_step(port, seconds, name)
            save_step(folder, name, rec)
            meta['steps'].append({'name': name, 'start_ns': rec['start_ns'], 'end_ns': rec['end_ns'],
                                  'open_ns': open_ns, 'opened_here': action != 'keep', 'stopped': rec['stopped']})
            write_json(os.path.join(folder, 'meta.json'), meta)
            if rec['stopped']:
                print('  Stopped (%s); keeping what was captured.' % rec['stopped'])
                break
            if name == 'far' and not split_packets(rec['data'])['packets']:
                print('  No valid packets: wrong port or baud rate, or another program is reading the port '
                      '(close the launcher / magnetometer_reader first).')
                break
    except KeyboardInterrupt:
        print('\n  Stopped with Ctrl-C; keeping what was captured.')
    except RuntimeError as exc:
        print('  ' + str(exc))
        meta['error'] = str(exc)
    finally:
        if port is not None:
            port.close()
        write_json(os.path.join(folder, 'meta.json'), meta)


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--port', '-p', default=None,
                        help='Serial port (default: the only ttyACM/ttyUSB port, else ask)')
    parser.add_argument('--baudrate', '-b', type=int, default=921600)
    parser.add_argument('--output-dir', default=os.path.join(_ROOT, 'data_collection', 'board_probe'),
                        help='Each run makes a YYYYmmdd_HHMMSS subfolder here')
    parser.add_argument('--listen-only', action='store_true',
                        help='Only steps 0 and 1: a 10 s check that the board streams')
    parser.add_argument('--replay', metavar='FOLDER', default=None,
                        help='No serial: re-analyse a saved probe folder and rewrite its report')
    return parser


def main(argv=None, prompt=input, open_port=open_serial_port, list_ports=list_port_info):
    args = build_parser().parse_args(argv)
    folder = args.replay
    if not folder:
        folder = os.path.join(args.output_dir, datetime.now().strftime('%Y%m%d_%H%M%S'))
        while os.path.exists(folder):
            folder += '_2'
        try:
            run_probe(args, folder, prompt, open_port, list_ports)
        except KeyboardInterrupt:
            pass
        if not os.path.exists(os.path.join(folder, 'meta.json')):
            print('\nStopped before anything was recorded.')
            return 1
    answers = write_report(analyze_folder(folder), folder)
    print('\n## Answers\n%s\n\nReport: %s' % (answers, os.path.abspath(os.path.join(folder, 'report.md'))))
    return 0


if __name__ == '__main__':
    sys.exit(main())
