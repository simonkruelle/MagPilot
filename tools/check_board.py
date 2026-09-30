#!/usr/bin/env python3
"""Check whether the board keeps an offset after a strong field, and which way the magnet points.

The board probe (tools/probe_board.py, 29 Sep 2026) found that after the stylus
had been on the cover, the readings kept a steady 5-31 uT offset although the
stylus was 1 m away. For writing that is tiny, but far above the board the
stylus field is only a few uT, so it matters for the tracking-error study.
This check measures it in a controlled way, in about 6 minutes:

  1 start   replug the board (fresh baseline), stylus away: 60 s of noise and drift
  2 weak    stylus 3 cm above the cover for 10 s, then 1 m away for 60 s
  3 strong  stylus on the cover above a middle sensor for 10 s, then away for 120 s
  4 replug  replug again (new baseline), stylus away: 30 s
  5 roll    stylus upright on the centre, rolled one full turn about its own axis

It answers: does a weak or a strong field leave an offset, how big, on which
sensors, does it fade, does a fresh baseline clear it, and does the magnet
point along the stylus or across it.

READ-ONLY like the probe: nothing is ever written to the board. The board
takes its baseline once, when the port is first opened after power-up, so the
check asks you to unplug and replug the USB cable (twice) and notices when you
do. Run it on the host: inside the container a replug is not visible.

Each run writes one folder named by local time (data_collection/ is gitignored):

    <output_dir>/<YYYYmmdd_HHMMSS>/meta.json         ports, open times, steps, cue times
    <output_dir>/<YYYYmmdd_HHMMSS>/<step>.bin        every raw byte received
    <output_dir>/<YYYYmmdd_HHMMSS>/<step>_reads.csv  t_ns,nbytes of each read
    <output_dir>/<YYYYmmdd_HHMMSS>/report.md         answers and a per-sensor table
    <output_dir>/<YYYYmmdd_HHMMSS>/report.json       all numbers

Close the launcher and magnetometer_reader first. Your access to the port must
survive a replug: the dialout group does (after a new login, or start the check
with sg dialout -c "python3 tools/check_board.py"); sudo chmod on the port does not.

Usage on the host:
  python3 tools/check_board.py
  python3 tools/check_board.py --replay data_collection/board_check/20260930_120000
"""

import argparse
import csv
import math
import os
import platform
import statistics
import sys
import time
from datetime import datetime

_TOOLS = os.path.dirname(os.path.abspath(__file__))
if _TOOLS not in sys.path:
    sys.path.insert(0, _TOOLS)

import probe_board as pb  # noqa: E402  (serial access, packet parsing, saturation checks)

TOOL_VERSION = '1.0'

# Step durations in seconds.
START_S = 60.0                  # noise and drift after a fresh baseline
HOLD_S = 10.0                   # stylus near the board
WEAK_AWAY_S = 60.0              # stylus away after the weak field
STRONG_AWAY_S = 120.0           # ...and after the strong field, long enough to see it fade
REPLUG_S = 30.0

# Analysis windows and thresholds (the report always shows the numbers).
SETTLE_AFTER_START_S = 2.0      # skip this much after the first non-zero packet of a start-up
REFERENCE_S = 30.0              # the last 30 s of 'start' are the zero reference
DRIFT_WINDOW_S = 10.0           # drift = last 10 s minus first 10 s of a step
AWAY_SETTLE_S = 10.0            # time to take the stylus away before an offset is measured
FADE_WINDOW_S = 20.0            # fading: the first and the last 20 s after the strong field
OFFSET_MIN_UT = 3.0             # a sensor offset (vector length) below this is noise and drift
ROLL_MIN_TILT_DEG = 20.0        # a magnet closer to vertical than this points along the stylus
ROLL_FULL_TURN_DEG = 270.0      # a direction that turned this far followed the roll
ROLL_MIN_FIELD_UT = 100.0       # in-plane field needed at a middle sensor to follow its turn

# Waiting for the replug.
PLUG_WAIT_S = 120.0             # how long to wait for the cable to come out or go back in
PLUG_POLL_S = 0.2
OPEN_RETRY_S = 5.0              # right after plug-in the port may not open yet

# Board layout: 4x4 sensors 35 mm apart (measured 30 Sep 2026: S1 to S13 is 105 mm),
# numbered column by column: S1-S4 at x = -52.5 mm from +y to -y, ..., S13-S16 at +52.5.
N_SENSORS = pb.N_SENSORS
N_FIELD = pb.N_FIELD_CHANNELS
PITCH_MM = 35.0
MIDDLE_SENSORS = (5, 6, 9, 10)  # S6, S7, S10, S11: the four sensors around the centre

AWAY = 'Take the stylus away, at least 1 m, and leave it there.'
FOLLOW = 'Hold the stylus at least 1 m from the board. After Enter, do what the >>> lines say.'

# (label, name, port action, instruction, cues). 'replug' waits for the cable to be
# pulled and put back, then opens the port; 'keep' records on the open port. Each cue
# (seconds, text) is shown when it starts; the cues of a step are recorded back to back.
STEPS = (
    ('1', 'start', 'replug',
     'The board restarts and takes a fresh baseline while the stylus is far away.',
     ((START_S, 'Leave the stylus at least 1 m away.'),)),
    ('2', 'weak', 'keep', FOLLOW,
     ((HOLD_S, 'Hold the stylus upright about 3 cm above the centre of the cover '
               '(do not touch it).'),
      (WEAK_AWAY_S, AWAY))),
    ('3', 'strong', 'keep', FOLLOW,
     ((HOLD_S, 'Hold the stylus upright on the cover, right above one of the four '
               'middle sensors.'),
      (STRONG_AWAY_S, AWAY))),
    ('4', 'replug', 'replug',
     'The board restarts again and takes a new baseline: does that clear the offset?',
     ((REPLUG_S, 'Leave the stylus at least 1 m away.'),)),
    ('5', 'roll', 'keep',
     'Stand the stylus upright on the cover over the centre. After Enter, do what the '
     '>>> lines say.',
     ((5.0, 'Hold it still, upright.'),
      (10.0, 'Roll it slowly one full turn about its own axis, tip in place, stylus upright.'),
      (5.0, 'Hold it still.'))),
)


# --------------------------------------------------------------------------
# Small helpers
# --------------------------------------------------------------------------

def sensor_xy_mm(s):
    """Board position of sensor index s (0-based)."""
    column, row = divmod(s, 4)
    return -1.5 * PITCH_MM + PITCH_MM * column, 1.5 * PITCH_MM - PITCH_MM * row


def window(rows, times, t0_ns, t1_ns):
    """The rows that arrived in [t0, t1)."""
    return [row for row, t in zip(rows, times) if t0_ns <= t < t1_ns]


def settled(rows, times):
    """Rows and times from SETTLE_AFTER_START_S after the first non-zero packet."""
    first = next((t for row, t in zip(rows, times) if any(row[:N_FIELD])), None)
    if first is None:
        return [], []
    keep = [(row, t) for row, t in zip(rows, times) if t >= first + SETTLE_AFTER_START_S * 1e9]
    return [row for row, _ in keep], [t for _, t in keep]


def offsets(rows, reference):
    """Per-channel mean(rows) - mean(reference) in uT, or None if a window is empty."""
    if not rows or not reference:
        return None
    return [a - b for a, b in zip(pb.column_means(rows), pb.column_means(reference))]


def sensor_lengths(vector):
    """Length of each sensor's (Bx, By, Bz) part of a 48-value vector."""
    return [math.sqrt(sum(v * v for v in vector[3 * s:3 * s + 3])) for s in range(N_SENSORS)]


def largest_fields(rows):
    """The largest |B| each sensor saw, in uT (None without rows)."""
    if not rows:
        return None
    return [max(math.sqrt(sum(v * v for v in row[3 * s:3 * s + 3])) for row in rows)
            for s in range(N_SENSORS)]


def unwrap_degrees(angles):
    """Angles without the jumps at +/-180, so a full turn adds up to 360."""
    out = []
    for a in angles:
        if out:
            while a - out[-1] > 180.0:
                a -= 360.0
            while a - out[-1] < -180.0:
                a += 360.0
        out.append(a)
    return out


def top_sensors(lengths, count=3):
    order = sorted(range(N_SENSORS), key=lambda s: -lengths[s])
    return ', '.join('S%d %s' % (s + 1, pb.fmt(lengths[s], 2)) for s in order[:count])


# --------------------------------------------------------------------------
# Analysis: pure functions over the saved steps
# --------------------------------------------------------------------------

def load_step(folder, name):
    """Raw bytes, reads, rows (54 floats) and arrival time (monotonic ns) of each row."""
    with open(os.path.join(folder, name + '.bin'), 'rb') as handle:
        data = handle.read()
    with open(os.path.join(folder, name + '_reads.csv'), newline='') as handle:
        reads = [(int(r['t_ns']), int(r['nbytes'])) for r in csv.DictReader(handle)]
    split = pb.split_packets(data)
    rows = [list(floats) for _, floats in split['packets']]
    times = []
    if reads:
        times = [t for t, _ in pb.arrival_of([last for last, _ in split['packets']], reads)]
    return data, reads, rows, times


def start_up(data, reads, open_ns):
    """Did the board restart when the port opened? (setup text or zero packets first)"""
    info, _ = pb.analyze_step(data, reads, open_ns, True)
    early_text = [t['text'].strip() for t in info['text']
                  if t['after_open_s'] is not None and t['after_open_s'] < pb.RESTART_TEXT_S]
    return {'fresh': bool(early_text) or info['leading_zero_packets'] > 0,
            'text': early_text, 'zero_packets': info['leading_zero_packets'],
            'first_packet_s': info['timing'].get('open_to_first_packet_s'),
            'packets': info['timing']['packets'], 'rate_hz': info['timing'].get('rate_hz')}


def drift(rows, times):
    """Per-channel mean of the last DRIFT_WINDOW_S minus the first (None if too short)."""
    if not times or times[-1] - times[0] < 2 * DRIFT_WINDOW_S * 1e9:
        return None
    early = window(rows, times, times[0], times[0] + DRIFT_WINDOW_S * 1e9)
    late = window(rows, times, times[-1] - DRIFT_WINDOW_S * 1e9, times[-1] + 1)
    return offsets(late, early)


def largest_channel(vector):
    """(channel name, value) with the largest |value|."""
    c = max(range(len(vector)), key=lambda i: abs(vector[i]))
    return pb.CHANNELS[c], vector[c]


def analyze_start(step, data, reads, rows, times):
    """Fresh start-up, noise and drift with the stylus away; returns the zero reference too."""
    out = {'start_up': start_up(data, reads, step.get('open_ns'))}
    rows, times = settled(rows, times)
    reference = window(rows, times, step['end_ns'] - REFERENCE_S * 1e9, step['end_ns'] + 1)
    out['reference_packets'] = len(reference)
    if reference:
        stds = [pb.mean_std([row[c] for row in reference])[1] for c in range(N_FIELD)]
        out['noise_xy_ut'] = statistics.median(v for c, v in enumerate(stds) if c % 3 != 2)
        out['noise_z_ut'] = statistics.median(stds[2::3])
    shift = drift(rows, times)
    if shift:
        out['drift_ut'] = shift
        out['drift_largest'] = largest_channel(shift)
    return out, reference


def analyze_exposure(step, rows, times, reference, far):
    """Largest fields, flat tops and the offset left after the stylus went away."""
    out = {'largest_ut': largest_fields(rows)}
    if rows:
        sat = pb.detect_saturation({step['name']: rows}, far)
        out['flat_tops'] = [f['channel'] for f in sat['flat_tops'] + sat['clipped']]
    segments = step.get('segments') or []
    if len(segments) < 2:
        return out, []
    away = segments[1]
    after = window(rows, times, away['start_ns'] + AWAY_SETTLE_S * 1e9, away['end_ns'] + 1)
    out['after_packets'] = len(after)
    shift = offsets(after, reference)
    if shift:
        out['offset_ut'] = shift
        out['offset_lengths_ut'] = sensor_lengths(shift)
    early = window(rows, times, away['start_ns'] + AWAY_SETTLE_S * 1e9,
                   away['start_ns'] + (AWAY_SETTLE_S + FADE_WINDOW_S) * 1e9)
    late = window(rows, times, away['end_ns'] - FADE_WINDOW_S * 1e9, away['end_ns'] + 1)
    long_enough = away['end_ns'] - away['start_ns'] >= (AWAY_SETTLE_S + 2 * FADE_WINDOW_S) * 1e9
    if early and late and reference and long_enough:
        out['fade_early_ut'] = sensor_lengths(offsets(early, reference))
        out['fade_late_ut'] = sensor_lengths(offsets(late, reference))
    return out, after


def analyze_replug(step, data, reads, rows, times):
    """After a new baseline: are the readings back at zero, and do they drift?"""
    out = {'start_up': start_up(data, reads, step.get('open_ns'))}
    rows, times = settled(rows, times)
    if rows:
        level = pb.column_means(rows)
        out['level_lengths_ut'] = sensor_lengths(level)
    shift = drift(rows, times)
    if shift:
        out['drift_ut'] = shift
        out['drift_largest'] = largest_channel(shift)
    return out


def analyze_roll(step, rows, times):
    """Firmware direction (tilt, azimuth turn) and the in-plane field turn during the roll."""
    segments = step.get('segments') or []
    if len(segments) < 2:
        return {}
    turn_rows = window(rows, times, segments[1]['start_ns'], segments[1]['end_ns'] + 1)
    if len(turn_rows) < 2:
        return {}
    mx, my, mz = pb.POSE['mx'], pb.POSE['my'], pb.POSE['mz']
    tilts = [math.degrees(math.acos(min(1.0, abs(row[mz])))) for row in turn_rows]
    azimuth = unwrap_degrees([math.degrees(math.atan2(row[my], row[mx])) for row in turn_rows])
    field_turns = []
    for s in MIDDLE_SENSORS:
        in_plane = statistics.median(math.hypot(row[3 * s], row[3 * s + 1]) for row in turn_rows)
        if in_plane < ROLL_MIN_FIELD_UT:
            continue                # too weak: the angle would follow the noise
        angles = unwrap_degrees([math.degrees(math.atan2(row[3 * s + 1], row[3 * s]))
                                 for row in turn_rows])
        field_turns.append(angles[-1] - angles[0])
    spans = [max(row[pb.POSE[a]] for row in turn_rows) - min(row[pb.POSE[a]] for row in turn_rows)
             for a in ('x', 'y')]
    return {'packets': len(turn_rows), 'tilt_deg': statistics.median(tilts),
            'direction_turn_deg': azimuth[-1] - azimuth[0],
            'field_turn_deg': statistics.median(field_turns) if field_turns else None,
            'position_range_mm': 1000.0 * max(spans)}


def analyze_folder(folder):
    """Analyse a saved check folder (the live run and --replay both use this)."""
    meta = pb.read_json(os.path.join(folder, 'meta.json'))
    steps = {step['name']: step for step in meta.get('steps', [])}
    loaded = {name: load_step(folder, name) for name in steps}
    r = {'tool_version': TOOL_VERSION, 'folder': os.path.abspath(folder), 'meta': meta,
         'recorded': list(steps)}
    reference, far = [], {}
    if 'start' in steps:
        r['start'], reference = analyze_start(steps['start'], *loaded['start'])
        far = pb.field_stats(reference)
    before_strong = reference
    if 'weak' in steps:
        rows, times = loaded['weak'][2:]
        r['weak'], after = analyze_exposure(steps['weak'], rows, times, reference, far)
        before_strong = after or reference
    if 'strong' in steps:
        rows, times = loaded['strong'][2:]
        r['strong'], _ = analyze_exposure(steps['strong'], rows, times, before_strong, far)
        r['strong']['reference'] = 'weak' if before_strong is not reference else 'start'
    if 'replug' in steps:
        r['replug'] = analyze_replug(steps['replug'], *loaded['replug'])
    if 'roll' in steps:
        r['roll'] = analyze_roll(steps['roll'], *loaded['roll'][2:])
    r['answers'] = build_answers(r)
    return r


# --------------------------------------------------------------------------
# Answers and report
# --------------------------------------------------------------------------

def start_up_text(info):
    if info['packets'] == 0:
        return 'no packets: wrong port, or another program is reading it'
    if not info['fresh']:
        return ('NO: no setup text and no zero packets, so the board did not restart and kept '
                'its old baseline (unplug the cable fully, then plug it back in)')
    signs = []
    if info['text']:
        signs.append('setup text')
    if info['zero_packets']:
        signs.append('%d zero packet(s)' % info['zero_packets'])
    return 'yes (%s)' % ', '.join(signs)


def offset_text(result, what):
    """'largest X uT (S#), ...' plus a verdict, for the offset after a field."""
    lengths = result.get('offset_lengths_ut')
    if lengths is None:
        return ('not measured: no readings from %d s or more after the stylus was taken away'
                % AWAY_SETTLE_S)
    above = [s for s in range(N_SENSORS) if lengths[s] >= OFFSET_MIN_UT]
    text = 'offset %s: largest %s uT, median %s uT over the 16 sensors; ' % (
        what, top_sensors(lengths), pb.fmt(statistics.median(lengths), 2))
    if above:
        return text + 'yes, %d sensor(s) at %s uT or more' % (len(above), pb.fmt(OFFSET_MIN_UT))
    return text + 'no clear offset (every sensor under %s uT)' % pb.fmt(OFFSET_MIN_UT)


def exposure_text(result):
    largest = result.get('largest_ut')
    if not largest:
        return 'no packets'
    s = max(range(N_SENSORS), key=lambda i: largest[i])
    text = 'largest field %s mT at S%d' % (pb.fmt(largest[s] / 1000.0, 3), s + 1)
    flat = result.get('flat_tops') or []
    if flat:
        text += '; X/Y or Z stopped rising (flat top) on %s' % ', '.join(flat[:4])
    return text


def fade_text(result):
    early, late = result.get('fade_early_ut'), result.get('fade_late_ut')
    if not early or not late:
        return ''
    s = max(range(N_SENSORS), key=lambda i: early[i])
    if early[s] < OFFSET_MIN_UT:
        return ''
    change = 100.0 * (late[s] - early[s]) / early[s]
    return ('; fading: S%d %s uT 10-30 s after, %s uT in the last 20 s (%+.0f %%)'
            % (s + 1, pb.fmt(early[s], 3), pb.fmt(late[s], 3), change))


def roll_text(roll):
    if not roll:
        return 'not measured'
    text = 'the magnet direction is %.0f deg from vertical' % roll['tilt_deg']
    if roll['tilt_deg'] < ROLL_MIN_TILT_DEG:
        return text + ': it points along the stylus, so rolling cannot turn it'
    text += ' and turned by %.0f deg during the roll (' % roll['direction_turn_deg']
    if roll['field_turn_deg'] is not None:
        text += 'the field at the middle sensors turned by %.0f deg; ' % roll['field_turn_deg']
    text += 'the stylus position moved %.0f mm)' % roll['position_range_mm']
    if abs(roll['direction_turn_deg']) >= ROLL_FULL_TURN_DEG:
        return text + ': the magnet points across the stylus (or sits tilted in it)'
    return text + (': less than a full turn, so either the roll was short or the magnet does '
                   'not follow it')


def build_answers(r):
    """(question, answer) pairs in plain English."""
    answers = []
    meta = r['meta']
    if meta.get('error'):
        answers.append(('Recording failed', meta['error']))
    if 'start' in r:
        s = r['start']
        text = 'fresh start-up at step 1: %s' % start_up_text(s['start_up'])
        if 'noise_xy_ut' in s:
            text += '; noise %s uT (X/Y), %s uT (Z)' % (pb.fmt(s['noise_xy_ut'], 2),
                                                       pb.fmt(s['noise_z_ut'], 2))
        if 'drift_largest' in s:
            name, value = s['drift_largest']
            text += '; drift over the minute: largest %s uT (%s)' % (pb.fmt(value, 2), name)
        answers.append(('Stylus away, fresh baseline', text))
    if 'weak' in r:
        answers.append(('After a weak field (3 cm above the cover)', '%s; %s' % (
            exposure_text(r['weak']), offset_text(r['weak'], '10-60 s after'))))
    if 'strong' in r:
        what = '10-120 s after (vs. before the strong field)'
        answers.append(('After a strong field (on the cover)', '%s; %s%s' % (
            exposure_text(r['strong']), offset_text(r['strong'], what), fade_text(r['strong']))))
    if 'replug' in r:
        p = r['replug']
        text = 'fresh start-up: %s' % start_up_text(p['start_up'])
        if 'level_lengths_ut' in p:
            text += '; readings after the new baseline: largest %s uT' % top_sensors(
                p['level_lengths_ut'], 1)
        if 'drift_largest' in p:
            name, value = p['drift_largest']
            text += '; drift over 30 s: largest %s uT (%s)' % (pb.fmt(value, 2), name)
        answers.append(('Replug (new baseline)', text))
    if 'roll' in r:
        answers.append(('Magnet direction (roll)', roll_text(r['roll'])))
    missing = [name for _, name, _, _, _ in STEPS if name not in r['recorded']]
    if missing:
        answers.append(('Not recorded', ', '.join(missing)))
    return answers


def sensor_table(r):
    """One row per sensor: position, largest fields, flat tops and offsets."""
    def cell(result, key, s, scale=1.0, decimals=1):
        values = (result or {}).get(key)
        return '%.*f' % (decimals, values[s] * scale) if values else 'n/a'
    weak, strong = r.get('weak'), r.get('strong')
    flat = set()
    for f in (strong or {}).get('flat_tops') or []:
        flat.add(f.split('_')[0])
    lines = ['| sensor | x, y mm | weak: largest mT | weak: offset uT | strong: largest mT '
             '| strong: flat top | strong: offset uT | 10-30 s after | last 20 s |',
             '|---|---|---|---|---|---|---|---|---|']
    for s in range(N_SENSORS):
        x, y = sensor_xy_mm(s)
        lines.append('| S%d | %+.1f, %+.1f | %s | %s | %s | %s | %s | %s | %s |' % (
            s + 1, x, y, cell(weak, 'largest_ut', s, 0.001, 2), cell(weak, 'offset_lengths_ut', s),
            cell(strong, 'largest_ut', s, 0.001, 2),
            'yes' if 'Sensor%d' % (s + 1) in flat else '',
            cell(strong, 'offset_lengths_ut', s), cell(strong, 'fade_early_ut', s),
            cell(strong, 'fade_late_ut', s)))
    return lines


def write_report(r, folder):
    """Write report.md and report.json; return the answers as plain text for the terminal."""
    meta = r['meta']
    plain = '\n'.join('- %s: %s' % pair for pair in r['answers'])
    lines = ['# Board check %s' % os.path.basename(r['folder']), '',
             'Recorded %s on %s (check version %s). Offsets are vector lengths per sensor, '
             'against the stylus-away readings before the field.' % (
                 meta.get('started_local'), meta.get('port'), r['tool_version']),
             '', '## Answers', '']
    lines += ['- **%s**: %s' % pair for pair in r['answers']]
    lines += ['', '## Per sensor', ''] + sensor_table(r)
    lines += ['', '## Steps', '', '| step | s | packets | cues |', '|---|---|---|---|']
    for step in meta.get('steps', []):
        cues = '; '.join('%.0f s: %s' % ((c['end_ns'] - c['start_ns']) / 1e9, c['cue'])
                         for c in step.get('segments', []))
        label = step['name'] + (' (stopped: %s)' % step['stopped'] if step.get('stopped') else '')
        lines.append('| %s | %.1f | %s | %s |' % (
            label, (step['end_ns'] - step['start_ns']) / 1e9, step.get('packets', 'n/a'), cues))
    with open(os.path.join(folder, 'report.md'), 'w', encoding='utf-8') as handle:
        handle.write('\n'.join(lines) + '\n')
    pb.write_json(os.path.join(folder, 'report.json'), r)
    return plain


# --------------------------------------------------------------------------
# Recording (only reads from the port, like the probe)
# --------------------------------------------------------------------------

def find_board(device, serial_number, list_ports):
    """The board's port now, or None: by USB serial number if known, else the same path."""
    if serial_number:
        for p in list_ports():
            if p.get('serial_number') == serial_number:
                return p['device']
        return None
    return device if os.path.exists(device) else None


def wait_for(condition, seconds, sleep):
    """Poll condition() until it returns something truthy; return that (or a falsy value)."""
    deadline = time.monotonic() + seconds
    while True:
        value = condition()
        if value or time.monotonic() >= deadline:
            return value
        sleep(PLUG_POLL_S)


def replug(device, serial_number, list_ports, prompt, sleep):
    """Wait until the board is unplugged and plugged back in; return its port."""
    print('  >>> Unplug the board\'s USB cable now.')
    if not wait_for(lambda: find_board(device, serial_number, list_ports) is None,
                    PLUG_WAIT_S, sleep):
        raise RuntimeError('The board was not unplugged within %d s. (Inside the container a '
                           'replug is not visible: run the check on the host.)' % PLUG_WAIT_S)
    print('  >>> Unplugged. Put the stylus at least 1 m away (another room is best), '
          'then plug the board back in.')
    back = wait_for(lambda: find_board(device, serial_number, list_ports), PLUG_WAIT_S, sleep)
    if not back:
        raise RuntimeError('The board did not come back within %d s after unplugging.'
                           % PLUG_WAIT_S)
    prompt('  Plugged in as %s. With the stylus at least 1 m away, press Enter: the board '
           'takes its baseline when the port opens > ' % back)
    return back


def open_after_plug(open_port, device, baudrate, sleep):
    """Open the port, retrying for OPEN_RETRY_S (just after plug-in it may not be ready)."""
    deadline = time.monotonic() + OPEN_RETRY_S
    while True:
        open_ns = time.monotonic_ns()
        try:
            return open_port(device, baudrate), open_ns
        except RuntimeError:
            if time.monotonic() >= deadline:
                raise
            sleep(0.5)


def record_cues(port, name, cues, flush):
    """Record each cue's seconds back to back on one open port, showing the cue first."""
    parts, segments = [], []
    for i, (seconds, text) in enumerate(cues):
        print('  %s>>> %s' % ('\a' if i else '', text))
        rec = pb.record_step(port, seconds, name, flush=flush and i == 0)
        parts.append(rec)
        segments.append({'cue': text, 'start_ns': rec['start_ns'], 'end_ns': rec['end_ns']})
        if rec['stopped']:
            break
    return {'data': b''.join(p['data'] for p in parts),
            'reads': [read for p in parts for read in p['reads']],
            'start_ns': parts[0]['start_ns'], 'end_ns': parts[-1]['end_ns'],
            'stopped': next((p['stopped'] for p in parts if p['stopped']), None),
            'stale_bytes': parts[0]['stale_bytes'], 'segments': segments}


def run_check(args, folder, prompt, open_port, list_ports, sleep):
    """Identify the port, then record each step. Ctrl-C keeps what was captured."""
    print('MagPilot board check: offset after a strong field, and the magnet direction.')
    print('READ-ONLY: nothing is written to the board. About 6 minutes; you will unplug and '
          'replug the board twice. Ctrl-C stops at any time and keeps what was recorded.')
    print('\nStep 0 identify:')
    ports = list_ports()
    pb.print_ports(ports)
    device = args.port or pb.choose_port(ports, prompt)
    real = os.path.basename(os.path.realpath(device))
    info = next((p for p in ports if os.path.basename(p['device']) == real), None)
    serial_number = (info or {}).get('serial_number')
    print('  -> using %s: %s' % (device, pb.board_answer(info)['text']))
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
    pb.write_json(os.path.join(folder, 'meta.json'), meta)
    port = None
    try:
        for label, name, action, instruction, cues in STEPS:
            print('\nStep %s %s (%d s): %s' % (label, name, sum(s for s, _ in cues), instruction))
            open_ns = None
            if action == 'replug':
                if port is not None:
                    port.close()
                    port = None
                device = replug(device, serial_number, list_ports, prompt, sleep)
                port, open_ns = open_after_plug(open_port, device, args.baudrate, sleep)
                meta['openings'].append({'step': name, 'device': device, 'open_ns': open_ns})
            else:
                prompt('  Press Enter to start > ')
            rec = record_cues(port, name, cues, flush=action == 'keep')
            pb.save_step(folder, name, rec)
            packets = len(pb.split_packets(rec['data'])['packets'])
            meta['steps'].append({'name': name, 'label': label, 'start_ns': rec['start_ns'],
                                  'end_ns': rec['end_ns'], 'open_ns': open_ns,
                                  'opened_here': action == 'replug', 'packets': packets,
                                  'segments': rec['segments'], 'stale_bytes': rec['stale_bytes'],
                                  'stopped': rec['stopped']})
            pb.write_json(os.path.join(folder, 'meta.json'), meta)
            if rec['stopped']:
                print('  Stopped (%s); keeping what was captured.' % rec['stopped'])
                break
            if action == 'replug':
                check = start_up(rec['data'], rec['reads'], open_ns)
                if not check['fresh']:
                    print('  Warning: fresh start-up %s.' % start_up_text(check))
    except (KeyboardInterrupt, EOFError) as exc:
        if isinstance(exc, EOFError):
            print('\n  Input closed (no keyboard). Keeping what was captured.')
        else:
            print('\n  Stopped with Ctrl-C; keeping what was captured.')
    except RuntimeError as exc:
        print('  ' + str(exc))
        meta['error'] = str(exc)
    finally:
        if port is not None:
            port.close()
        pb.write_json(os.path.join(folder, 'meta.json'), meta)


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------

def build_parser():
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--port', '-p', default=None,
                        help='Serial port (default: the only ttyACM/ttyUSB port, else ask)')
    parser.add_argument('--baudrate', '-b', type=int, default=921600)
    default_output = os.path.join(os.path.dirname(_TOOLS), 'data_collection', 'board_check')
    parser.add_argument('--output-dir', default=default_output,
                        help='Each run makes a YYYYmmdd_HHMMSS subfolder here')
    parser.add_argument('--replay', metavar='FOLDER', default=None,
                        help='No serial: re-analyse a saved check folder and rewrite its report')
    return parser


def main(argv=None, prompt=input, open_port=pb.open_serial_port, list_ports=pb.list_port_info,
         sleep=time.sleep):
    args = build_parser().parse_args(argv)
    folder = args.replay
    if folder and not os.path.exists(os.path.join(folder, 'meta.json')):
        raise SystemExit('%s is not a check folder (no meta.json).' % folder)
    if not folder:
        folder = os.path.join(args.output_dir, datetime.now().strftime('%Y%m%d_%H%M%S'))
        while os.path.exists(folder):
            folder += '_2'
        try:
            run_check(args, folder, prompt, open_port, list_ports, sleep)
        except (KeyboardInterrupt, EOFError):
            pass
        if not os.path.exists(os.path.join(folder, 'meta.json')):
            print('\nStopped before anything was recorded.')
            return 1
    meta = pb.read_json(os.path.join(folder, 'meta.json'))
    if not meta.get('steps'):
        print('\nNothing was recorded%s' % (': ' + meta['error'] if meta.get('error') else '.'))
        return 1
    answers = write_report(analyze_folder(folder), folder)
    print('\nAnswers\n%s\n\nReport: %s\nPlease zip this folder and send it: %s' % (
        answers, os.path.abspath(os.path.join(folder, 'report.md')), os.path.abspath(folder)))
    return 1 if meta.get('error') else 0


if __name__ == '__main__':
    sys.exit(main())
